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

### Camera requirements

- **Intrinsics must be configured.** The detector uses `focal_x_px`, `focal_y_px`, `center_x_px`, `center_y_px` from the camera's `intrinsic_parameters`. With unset or zero-valued intrinsics, pose estimation is meaningless.
- **JPEG output.** The detection loop converts the first JPEG-mime image returned by `get_images()` to grayscale. Cameras that publish only PNG or raw frames will be skipped.
- **The camera must have a frame configured** in the frame system (parent, translation, orientation). Without that, the renderer has no way to place the camera in the world and the tag transforms will be orphaned.

### How tags appear in the 3D scene

Each detected tag is published as **two** world state transforms:

- `april_tag_<id>_origin` — a tiny 1mm marker cube at the tag's bottom-left corner. This is what carries the visible axes triad. Its frame has X right, Y up, and Z pointing into the tag (away from the camera).
- `april_tag_<id>` — a flat `tag_width_mm × tag_width_mm × 1mm` box centered on the tag, covering the printed face.

Two transforms are needed because the 3D scene viewer ignores the `Geometry.center` field on a `Transform` — the geometry is always drawn at the frame's pose. To show both a corner-anchored axes triad and a tag-area geometry, the two anchors must live on separate frames.

The bottom-left corner is computed from the detector's reported pose by translating along `R · (-w/2, +w/2, 0)`. dt_apriltags uses Y-down tag-local coordinates, so `+y` is physically down. The display orientation rotates the apriltag frame 180° around X, which gives a final convention of X right, Y up, Z into the tag.

### Event semantics

The detection loop runs at `detection_rate_hz` and emits one of three change types per UUID per cycle:

- `ADDED` — UUID is being seen for the first time.
- `UPDATED` — UUID was present in the previous cycle and is still present (pose may have changed).
- `REMOVED` — UUID was present in the previous cycle and is no longer detected.

There is no movement threshold and no flicker debouncing: every cycle in which a tag is visible emits an `UPDATED` event for both of its UUIDs, and a single missed frame produces immediate `REMOVED` events followed by `ADDED` when it reappears.

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

## Build

```sh
make module.tar.gz
```
