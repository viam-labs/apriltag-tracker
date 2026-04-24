# apriltag-tracker

A Viam module that continuously detects AprilTags in a camera feed and publishes each detected tag as a world state transform. Configured tags appear as flat rectangular geometries in the Viam app's **3D scene** tab, positioned in the camera's reference frame.

> **Note:** The [`viam-labs/apriltag`](https://github.com/viam-labs/apriltag) module is poll-based — it implements the `PoseTracker` component, and clients call `get_poses` to retrieve current detections. This module is continuous: a background loop pushes detection events to the world state store, so tags render in the 3D scene without client-side polling.

## Model

| Model | API | Description |
| ----- | --- | ----------- |
| `shrews-testing:apriltag-tracker:april_tag_visualizer` | `rdk:service:world_state_store` | Continuous AprilTag detector that publishes each detected tag as a world state transform. |

## Configuration

Add the service to your machine's configuration with the following attribute schema:

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

- **Intrinsics must be configured.** The detector uses `focal_x_px`, `focal_y_px`, `center_x_px`, `center_y_px` from the camera's `intrinsic_parameters`. With unset (zero-valued) intrinsics, the resulting pose is meaningless.
- **JPEG output.** The detection loop converts the first JPEG-mime image returned by `get_images()` to grayscale. Cameras that publish only PNG or raw frames will be skipped.

## How tags appear in the 3D scene

Each detected tag is published as a `Transform` with:

- `uuid = str(tag_id).encode()` — stable across detections.
- `reference_frame = "tag-<id>"` — the tag's own frame name.
- `pose_in_observer_frame.reference_frame = <camera_name>` — the tag pose is expressed in the camera frame.
- `physical_object` — a flat `RectangularPrism` of `tag_width_mm × tag_width_mm × 1mm`, labelled `tag-<id>`.

For the geometry to land in the world the **camera component must have a frame configured** in the frame system (parent, translation, orientation). Without that, the renderer has no way to place the camera in the world and the tag transforms will be orphaned.

### Event semantics

The module's detection loop runs at `detection_rate_hz` and emits one of three change types per tag per cycle:

- `ADDED` — first detection of a tag id.
- `UPDATED` — tag id was present in the previous cycle and is still present (pose may have changed).
- `REMOVED` — tag id was present in the previous cycle and is no longer detected.

There is no movement threshold and no flicker debouncing: every cycle in which a tag is visible emits an `UPDATED` event, and a single missed frame produces an immediate `REMOVED` followed by `ADDED` when it reappears. If this becomes a problem in practice, see the design notes in `CLAUDE.md` for where to add either knob.

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
