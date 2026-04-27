# apriltag-tracker

A Viam module that continuously detects AprilTags in a camera feed and exposes them in two complementary ways:

- **3D scene view** — each detected tag is published as a world state transform, so it renders as a flat box in the Viam app's 3D scene tab, positioned in the camera's reference frame.
- **2D overlay view** — a wrapping camera component returns annotated JPEG frames with each tag's four corners drawn as a polygon and the tag id labelled at the center.

> **Note:** The [`viam-labs/apriltag`](https://github.com/viam-labs/apriltag) module is poll-based — it implements the `PoseTracker` component, and clients call `get_poses` to retrieve current detections. This module is continuous: detection runs in a background loop and pushes events to subscribers, so tags render in the 3D scene without client-side polling.

## Models

| Model | API | Description |
| ----- | --- | ----------- |
| `shrews-testing:apriltag-tracker:april_tag_visualizer` | `rdk:service:world_state_store` | Continuous AprilTag detector that publishes detected tags as world state transforms for the 3D scene tab. |
| `shrews-testing:apriltag-tracker:overlay_camera` | `rdk:component:camera` | Camera that wraps another camera and overlays each detected tag's four corners (true rotated/skewed quad) and id on the frame. |

The two models are independent — install the module once and add either or both to your machine. Both depend on a JPEG-producing source camera.

## 3D scene view — `april_tag_visualizer`

Add the service to your machine's configuration:

```json
{
  "camera_name": "camera-1",
  "tag_family": "tag36h11",
  "tag_width_mm": 29.5,
  "detection_rate_hz": 5.0
}
```

### Attributes

| Name | Type | Inclusion | Default | Description |
| ---- | ---- | --------- | ------- | ----------- |
| `camera_name` | string | **Required** | — | Name of the camera component to read frames from. The service declares this as an implicit dependency. |
| `tag_family` | string | **Required** | — | AprilTag family to detect, e.g. `tag36h11`, `tag25h9`, `tagStandard41h12`. Comma-separated values are accepted to detect multiple families simultaneously. |
| `tag_width_mm` | float | **Required** | — | Physical tag width corner-to-corner in millimeters. Required for metric pose estimation; an incorrect value produces tags at the wrong distance. |
| `detection_rate_hz` | float | Optional | `5.0` | Detection loop rate. Each cycle pulls a frame, runs detection, and emits diff events to subscribers. Higher rates increase camera and CPU load. |
| `centroid_alpha` | float | Optional | `1.0` | Opacity of the centroid box geometry, between `0.0` (fully transparent) and `1.0` (fully opaque). At values below `1.0` the box is published with `metadata: {"opacity": <alpha>}` so the 3D scene viewer can render it translucent and let underlying point clouds / scene content show through. The BL-corner origin marker stays opaque regardless. *Note:* requires a 3D scene viewer that honors `Transform.metadata.opacity`. |
| `motion_service_name` | string | Optional | `"builtin"` | Name of the motion service the visualizer will use to compose camera-frame poses to world frame in `do_command`'s `get_pose` response. Declared as an optional implicit dependency, so if no service with this name is configured on the machine, world-frame poses are returned as `null` and a warning is logged at reconfigure time. |

### Camera requirements

- **Intrinsics must be configured.** The detector uses `focal_x_px`, `focal_y_px`, `center_x_px`, `center_y_px` from the camera's `intrinsic_parameters`. With unset or zero-valued intrinsics, pose estimation is meaningless.
- **JPEG output.** The detection loop converts the first JPEG-mime image returned by `get_images()` to grayscale. Cameras that publish only PNG or raw frames will be skipped.
- **The camera must have a frame configured** in the frame system (parent, translation, orientation). Without that, the renderer has no way to place the camera in the world and the tag transforms will be orphaned.

#### Multi-sensor cameras and `extrinsic_parameters`

Some cameras report intrinsics for one sensor (e.g. color) while treating a different sensor as the frame origin (e.g. depth). Viam's RealSense module does this: `get_properties` returns the color stream's intrinsics when `main_sensor` is color, but the camera's reference frame is anchored at the depth left imager. Without compensation, tag poses derived from color-stream intrinsics arrive in the color sensor's frame and end up offset by roughly 15 mm in X relative to where Viam's frame system expects them. This module reads `properties.extrinsic_parameters.translation` — which the RealSense module populates with the color→depth offset in millimeters — and adds it to every emitted tag pose, so the rendered position lands at the correct camera reference origin automatically. For cameras that leave `extrinsic_parameters` unset, the offset is `(0, 0, 0)` and the correction is a no-op.

### How tags appear in the 3D scene

![Two AprilTags rendered in the 3D scene tab](3d_scene.png)

Two AprilTags (ids 20 and 21) detected and rendered in the **3D SCENE** tab, overlaid on the realsense point cloud. The picture-in-picture in the corner shows the `apriltag-overlay` camera feed; the sidebar lists the four world-state-store entries per detection cycle (centroid + origin × two tags).

Each detected tag is published as **two** world state transforms per cycle:

- `april_tag_<id>_origin_<epoch_ms>` — a 10 mm marker cube at the tag's bottom-left corner. This is what carries the visible axes triad. Its frame has X right, Y up, and Z pointing into the tag (away from the camera).
- `april_tag_<id>_centroid_<epoch_ms>` — a flat `tag_width_mm × tag_width_mm × 1 mm` box centered on the tag, covering the printed face.

The visible **labels** in the 3D scene UI are unsuffixed — `april_tag_21_origin` and `april_tag_21_centroid` — so the displayed name doesn't churn even though the underlying UUID does.

Two transforms are needed because the 3D scene viewer ignores the `Geometry.center` field on a `Transform`: the geometry is always drawn at the frame's pose. To show both a corner-anchored axes triad and a tag-area geometry, the two anchors must live on separate frames.

The bottom-left corner is computed from the detector's reported pose by translating along `R · (-w/2, +w/2, 0)`. dt_apriltags uses Y-down tag-local coordinates, so `+y` is physically down. The display orientation rotates the apriltag frame 180° around X, which gives a final convention of X right, Y up, Z into the tag.

### Event semantics

The detection loop runs at `detection_rate_hz`. Each cycle stamps every UUID with a fresh epoch-millisecond suffix and emits, in order:

1. `REMOVED` for every UUID present in the previous cycle.
2. `ADDED` for every UUID present in the new cycle.

`UPDATED` is intentionally never emitted. The 3D scene renderer caches by UUID and ignores `ADDED` for any UUID it has previously seen — even after a `REMOVED` — so geometries would appear frozen if we used stable UUIDs with `UPDATED`. Per-cycle UUID timestamping (the same workaround pallet-config uses) ensures every cycle's emit looks fresh to the renderer and the geometries track motion in real time. The visible labels stay stable so the 3D scene UI remains readable.

There is no movement threshold and no flicker debouncing: every cycle a tag is visible re-emits its two UUIDs at the new timestamp, and a missed frame produces a clean `REMOVED` for that tag's last-seen UUIDs without an immediate `ADDED`.

### Querying via `do_command`

Other modules and SDK clients can query the visualizer directly with `do_command`. The dispatch key is `"command"`. All commands include a `timestamp_ms` field on success — the epoch-millisecond suffix that's also embedded in the current cycle's UUIDs, so callers can correlate a query response with a specific detection cycle.

#### `list_tags` — currently detected tag ids

```python
await visualizer.do_command({"command": "list_tags"})
```
```json
{
  "tags": [20, 21],
  "timestamp_ms": 1777324893012
}
```

Sorted unique tag ids currently detected. Empty list if no tags are visible.

#### `list_uuids` — currently published UUIDs

```python
await visualizer.do_command({"command": "list_uuids"})
```
```json
{
  "uuids": [
    "april_tag_20_origin_1777324893012",
    "april_tag_20_centroid_1777324893012",
    "april_tag_21_origin_1777324893012",
    "april_tag_21_centroid_1777324893012"
  ],
  "timestamp_ms": 1777324893012
}
```

Two UUIDs per detected tag. The epoch-ms suffix changes every cycle (see "Event semantics") — these UUIDs are only valid for the cycle whose `timestamp_ms` matches.

#### `get_pose` — full pose for one tag

```python
await visualizer.do_command({"command": "get_pose", "tag_id": 21})
```
```json
{
  "tag_id": 21,
  "origin": {
    "camera_frame": {"x": -100.5, "y": 100.5, "z": 1500.0,
                     "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 180.0},
    "world_frame": {"x": -85.85, "y": 100.68, "z": 1500.34,
                    "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 180.0}
  },
  "centroid": {
    "camera_frame": {"x": 0.0, "y": 0.0, "z": 1500.0,
                     "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 180.0},
    "world_frame": {"x": 14.65, "y": 0.18, "z": 1500.34,
                    "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 180.0}
  },
  "camera_name": "realsense-cam",
  "timestamp_ms": 1777324893012
}
```

Both the **origin marker** (BL corner) and the **centroid** (tag center) come back, each in the camera reference frame and in the world frame. World-frame composition is delegated to the configured motion service (`motion_service_name` attribute, default `"builtin"`); if motion isn't available the corresponding `world_frame` is `null` and a warning is logged at reconfigure time. All translations are in millimeters.

If the tag id is not currently detected:

```json
{
  "tag_id": 21,
  "origin": null,
  "centroid": null
}
```

Either `origin` or `centroid` may individually be `null` if only one of the two transforms is in the current state (rare — they're emitted together).

The `tag_id` argument may be an int or a numeric string (`"21"` is accepted). A non-numeric value raises.

#### `get_transforms` — full snapshot of every current transform

```python
await visualizer.do_command({"command": "get_transforms"})
```
```json
{
  "transforms": [
    {
      "uuid": "april_tag_21_origin_1777324893012",
      "label": "april_tag_21_origin",
      "observer_frame": "realsense-cam",
      "pose": {"x": -100.5, "y": 100.5, "z": 1500.0,
               "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 180.0},
      "metadata": {}
    },
    {
      "uuid": "april_tag_21_centroid_1777324893012",
      "label": "april_tag_21_centroid",
      "observer_frame": "realsense-cam",
      "pose": {"x": 0.0, "y": 0.0, "z": 1500.0,
               "o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 180.0},
      "metadata": {"opacity": 0.4}
    }
  ],
  "timestamp_ms": 1777324893012
}
```

Returns every transform in the current cycle, with the same fields the world state store stream would deliver. `metadata` mirrors what's on the wire (e.g. `{"opacity": 0.4}` on centroid entries when `centroid_alpha` is below 1.0). Useful for verifying that a renderer-side issue isn't on this module's side. Poses are in the camera reference frame; to compose to world for any of them, use `get_pose` (which does the composition) or call your motion service directly.

#### No `command` — debug snapshot

```python
await visualizer.do_command({})
```

Returns a snapshot useful for diagnosing why detection or rendering isn't working: loop liveness, exception state, last cycle timestamp and error, camera intrinsics, distortion parameters, sensor extrinsic offset (the RealSense color↔depth correction we automatically apply), mime types the camera returned in the most recent cycle, the gray-image shape, current tracked UUIDs, and the resolved config attributes. See the source for the full field list.

## 2D overlay view — `overlay_camera`

Add a camera component to your machine:

```json
{
  "name": "apriltag-overlay",
  "api": "rdk:component:camera",
  "model": "shrews-testing:apriltag-tracker:overlay_camera",
  "attributes": {
    "camera_name": "camera-1",
    "tag_family": "tag36h11"
  }
}
```

### Attributes

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `camera_name` | string | **Required** | Name of the source camera component to wrap. The overlay camera reads JPEG frames from this camera, runs detection, and re-encodes with corners drawn. |
| `tag_family` | string | **Required** | AprilTag family to detect. Comma-separated values are accepted. |

### How the overlay looks

The overlay camera shows up in the **CONTROL** tab as a regular camera. Each detected tag's four corners are drawn as a green polygon (rotated and skewed to match the tag's actual perspective in the image) and the tag id is labelled in red at the tag center.

![overlay_camera in the CONTROL tab](overlay_card.png)

The wrapper proxies `get_properties` and `get_point_cloud` to the source camera unchanged, and only annotates JPEG-mime images. Non-JPEG images (e.g. depth) pass through untouched. Detection runs on each `get_image` / `get_images` call rather than from a continuous loop, since cameras are pulled by clients on demand.

## Generating tags

The example `tag36h11` PDF from the predecessor module is suitable for getting started: [`tag36h11_1-30.pdf`](https://github.com/viam-labs/apriltag/blob/main/tag36h11_1-30.pdf). Online generators such as [shiqiliu-67's apriltag-generator](https://shiqiliu-67.github.io/apriltag-generator/) can produce other families and sizes. For details on the AprilTag specification, see the [AprilRobotics repo](https://github.com/aprilrobotics/apriltag).

## Local development

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m src.main
```

The module expects to be launched by `viam-server` via `run.sh`, which provisions a virtualenv and installs dependencies before exec-ing the entrypoint.

## Tests

```sh
make test
```

Installs `requirements-dev.txt` (pytest + pytest-asyncio on top of runtime deps) and runs the suite under `tests/`. The tests construct fake apriltag detections and exercise the per-tag transform math, config validation, and the `do_command` dispatch surface — no live camera required. Tests must be run from the repo root because `spatialmath.py` loads `libviam_rust_utils-linux_<arch>.so` via a relative path.

## Build

```sh
make module.tar.gz
```
