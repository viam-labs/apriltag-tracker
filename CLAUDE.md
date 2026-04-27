# CLAUDE.md — apriltag-tracker

Operational context for future agents working on this repo. Read alongside `README.md` (user-facing).

## What this is

A Viam module that implements two AprilTag-related models:

- `shrews-testing:apriltag-tracker:april_tag_visualizer` (API `rdk:service:world_state_store`) — runs a continuous AprilTag detection loop against a configured camera and publishes each detected tag as a `Transform` so it renders in the Viam app's 3D scene tab.
- `shrews-testing:apriltag-tracker:overlay_camera` (API `rdk:component:camera`) — wraps a source camera and returns annotated JPEGs with each detected tag's four corners drawn as a polygon (rotated/skewed to match the actual tag in the image) and id labelled at the tag center. Provides the 2D companion view to the 3D visualizer.

A separate module, [`viam-labs/apriltag`](https://github.com/viam-labs/apriltag), exposes the same detection capability via the `PoseTracker` component — clients poll `get_poses` to retrieve current detections. This module is the continuous-push counterpart: a background loop emits `REMOVED` for every UUID from the previous cycle plus `ADDED` for every UUID in the new cycle, so subscribers see a fresh state each tick.

## File layout

```
src/main.py          # Module entrypoint. Imports model classes so they self-register, then runs the registry.
src/visualizer.py    # AprilTagVisualizer — the WorldStateStore implementation (3D scene).
src/overlay_camera.py # OverlayCamera — Camera implementation that returns annotated JPEGs (2D scene).
src/spatialmath.py   # quaternion -> Viam orientation vector via libviam_rust_utils ctypes.
libviam_rust_utils-linux_*.so   # Native helpers for the orientation-vector conversion.
meta.json            # Module metadata. Two model entries.
requirements.txt     # Python deps. viam-sdk must be a version that exposes viam.services.worldstatestore.
requirements-dev.txt # Adds pytest and pytest-asyncio on top of runtime deps for the tests/.
pytest.ini           # asyncio_mode = auto and testpaths = tests.
Makefile             # `make module.tar.gz` packages the module for upload; `make test` runs pytest.
run.sh               # viam-server entrypoint. Creates venv, installs deps, exec's `python -m src.main`.
tests/               # pytest suite for visualizer.py — exercises pose math, config validation, and do_command.
```

## Tests

`make test` from the repo root installs dev deps and runs the suite. Tests bypass `EasyResource.new()` by constructing the visualizer with `__new__` + manual `__init__`, then poking the attributes the methods under test read (camera name, tag width, alpha, sensor offset). Fake "tag" inputs are `SimpleNamespace`s mimicking the `tag_id`/`pose_R`/`pose_t` fields `_build_transforms` reads from `dt_apriltags.Detection`.

The detection loop, RealSense communication, and renderer behavior are intentionally out of scope — those need integration testing against real hardware. The unit tests cover the deterministic, computable parts: BL corner math, Rx180 display rotation, sensor-offset application, opacity metadata wiring, UUID/label naming, validate_config, and the do_command dispatch table. **Tests must run from the repo root** because `spatialmath.py` loads `libviam_rust_utils-linux_<arch>.so` via a relative path; running pytest from `tests/` will fail to import.

## Releasing

The current published-on-registry version is recorded in the `VERSION` file at the repo root (one line, bare semver, no `v` prefix). The Makefile's `upload` target reads it:

```sh
make upload
```

…which runs `make test`, builds `module.tar.gz`, then pushes via `viam module upload --version=$(cat VERSION) --platform=linux/any`. Tests are a hard dependency on the upload target — if any of the 33 unit tests fail the upload won't run. **Never push the module without running the tests** — the registry rejects duplicate version uploads, so a bad push then needs both a code fix and a version bump to recover.

Workflow for a release:

1. Edit `VERSION` to the new semver (registry rejects duplicates, so always bump).
2. Commit the `VERSION` change and any code/doc changes together.
3. `make upload`.
4. `git push` the commit so the source repo reflects the published version.

## Architecture

### Lifecycle

1. `viam-server` runs `run.sh`, which exec's `python -m src.main`.
2. `Module.run_from_registry()` discovers `AprilTagVisualizer` (registered via the `EasyResource` mixin) and serves it.
3. On initial resource creation, the framework calls `new(config, dependencies)`. `EasyResource.new` only constructs the instance — for service models the framework does **not** auto-call `reconfigure`, so our `new` calls it explicitly. See "Notes" below.
4. On subsequent machine reconfigure events, the framework calls `validate_config` then `reconfigure` directly.
5. `reconfigure` cancels any prior detect-loop task, clears `_detected`, and starts a fresh `_detect_loop` task.
6. `close` cancels the detect-loop task on shutdown.

### Detection loop

`_detect_loop` runs forever, calling `_detect_once` every `1.0 / detection_rate_hz` seconds. Each cycle:

1. Pulls camera intrinsics from `camera.get_properties()`.
2. Pulls a frame from `camera.get_images()`, picks the first JPEG, converts to grayscale via PIL + `cv2.cvtColor`.
3. Runs `dt_apriltags.Detector.detect(...)` with `estimate_tag_pose=True` and `tag_size = 0.001 * tag_width_mm` (the apriltag library expects meters).
4. Builds a `Transform` for each tag.
5. Under `_lock`: stamps `self._cycle_ts` with the current epoch ms, broadcasts a `REMOVED` event for every UUID in the previous cycle, then an `ADDED` event for every UUID in the new cycle. No `UPDATED` events — see "UUID strategy" below.
6. Replaces `self._detected` with `new_state`.

Exceptions from `_detect_once` are caught and logged at WARNING; the loop continues. `asyncio.CancelledError` propagates so `close` can shut the loop down cleanly.

### Subscriber fanout

`stream_transform_changes` is an async generator. Each subscriber:

1. Allocates an `asyncio.Queue(maxsize=256)` and appends it to `self._subscribers` under `_lock`.
2. Receives an initial burst of `ADDED` events for everything currently in `_detected`, so subscribers that join mid-stream see the current world.
3. Yields whatever the loop pushes into its queue.
4. On generator close (client disconnect, cancellation), removes its queue from `self._subscribers`.

`_broadcast` iterates `list(self._subscribers)` and `put_nowait`s the event into each queue. Full queues drop the event with a warning log — better than blocking the detect loop on a slow consumer.

### UUID strategy

UUIDs are **not** stable across detections. Each cycle stamps every emitted transform with the current epoch milliseconds (`self._cycle_ts`), producing UUIDs like `april_tag_21_centroid_1777324893012`. The diff is intentionally not a real diff — every cycle, we `REMOVED` everything in the previous state and `ADDED` everything in the new state.

This is the same workaround pallet-config uses, and it's required: the 3D scene renderer caches by UUID and drops `ADDED` for any UUID it has *ever* seen, even after a `REMOVED` in the same cycle. Confirmed empirically — we tried `UPDATED` (geometries froze), tried REMOVED+ADDED with stable UUIDs (geometries vanished after first cycle), and the only thing that produces real-time motion in the renderer is fresh UUIDs each cycle.

The visible **labels** stay unsuffixed (`april_tag_21_centroid`, `april_tag_21_origin`) so the displayed name in the 3D scene UI doesn't churn. Only the UUID and `reference_frame` carry the timestamp — the renderer keys on UUID, the user reads the label.

### Per-tag transforms

`_build_transforms(tag, ts_ms)` returns a list of two `Transform` protos for each detected tag, both timestamp-suffixed for the cycle:

1. **Origin marker** — UUID `april_tag_<id>_origin_<ts_ms>`, label `april_tag_<id>_origin`. A 10 mm cube placed at the tag's **bottom-left corner**. Its frame has X right, Y up, Z into the tag. Carries the user-visible axes triad. The cube is large enough that the renderer draws axes against it; 1 mm fell below the renderer's annotate-this-frame size threshold.
2. **Centroid** — UUID `april_tag_<id>_centroid_<ts_ms>`, label `april_tag_<id>_centroid`. A `tag_width_mm × tag_width_mm × 1 mm` box at the **tag center**, covering the printed face. When the `centroid_alpha` config attribute is below `1.0`, the centroid `Transform` is also emitted with `metadata = {"opacity": <alpha>}` so that 3D scene viewers honoring `Transform.metadata.opacity` render the box translucent. The BL-corner marker is always opaque.

Two transforms are required because **the 3D scene viewer ignores `Geometry.center`**: the box is always drawn at the frame's `pose_in_observer_frame.pose` regardless of any offset specified inside the geometry. Pallet-config hits the same constraint and uses the same workaround — set `pose_in_observer_frame.pose` to where the geometry should land and don't bother with `Geometry.center`. To get both a BL-anchored origin marker AND a tag-area geometry from a single detection, the two anchors must live on separate frames.

#### Pose math

The detector returns `pose_R` (3x3) and `pose_t` (3x1, in meters because we pass `tag_size = 0.001 * tag_width_mm`).

- **dt_apriltags uses Y-down tag-local coordinates** (image-coordinate convention), so the BL corner is at local `(-w/2, +w/2, 0)`, not `(-w/2, -w/2, 0)`. Getting this wrong puts the origin at the top-left.
- **`t_corner = t + R · (-w/2, +w/2, 0)`** — the BL corner expressed in camera frame.
- **`R_display = R · Rx180`** where `Rx180 = diag(1, -1, -1)`, rotating 180° around X. With the Y-down apriltag frame this yields a display frame with X right, Y up, Z into the tag.
- **`self._sensor_offset_mm`** (read from `properties.extrinsic_parameters.translation` each cycle) is added to both translations after multiplying by 1000 to convert meters → mm. This shifts the pose from the reported intrinsics' sensor frame (e.g. RealSense color sensor) into the camera's actual reference frame (e.g. RealSense depth left imager). For cameras with no extrinsic_parameters set the offset is zero.

Both transforms share the same `R_display` orientation and `pose_in_observer_frame.reference_frame = camera.name`. Only the translation differs (BL corner vs. tag center).

### `do_command` query interface

Dispatches on a `"command"` field in the input. Used by other modules / SDK clients that want a clean read of detection state without subscribing to `stream_transform_changes`:

- `list_tags` → `{tags: [int], timestamp_ms}` — sorted unique tag ids currently detected.
- `list_uuids` → `{uuids: [str], timestamp_ms}` — current UUIDs (with timestamp suffixes).
- `get_pose` with `tag_id: int` → `{tag_id, origin: {camera_frame, world_frame}, centroid: {camera_frame, world_frame}, camera_name, timestamp_ms}` — both the BL-corner origin marker and the tag-center centroid, each in the camera reference frame and in world. World-frame composition uses the configured motion service (`motion_service_name`, default `"builtin"`); if motion isn't available the corresponding `world_frame` is `null` and a warning was logged at reconfigure time. Returns `{tag_id, origin: null, centroid: null}` when the tag id isn't currently in `_detected`. Accepts numeric strings as well as ints for `tag_id`.
- `get_transforms` → `{transforms: [{uuid, label, observer_frame, pose, metadata}], timestamp_ms}` — full snapshot. `metadata` mirrors what's on the wire; centroid entries carry `{"opacity": <alpha>}` when `centroid_alpha < 1.0`. Useful for ruling out our side when the 3D scene viewer doesn't render an effect we expected.
- No `command` key → debug snapshot (loop liveness, last cycle timing, intrinsics, distortion params, sensor_offset_mm, mime types, current uuids, configured attributes).

Lookup helpers: `_tag_ids_from_detected` parses tag ids from the unsuffixed labels; `_pose_to_dict` flattens a `Pose` proto into a JSON-friendly dict; `_world_pose` composes a Transform's camera-frame pose into world via `motion.get_pose(component_name=tf.reference_frame, destination_frame="world", supplemental_transforms=[tf])` — passing the tag transform itself as a supplemental frame, so the motion service walks the augmented frame system rather than us re-implementing SE(3) composition.

### Overlay camera (`overlay_camera.py`)

A `Camera` component model that wraps a source camera and returns annotated JPEGs. Architecture is much simpler than the visualizer:

- **No background loop.** Detection runs synchronously inside each `get_image` / `get_images` call. Cameras are pulled by clients on demand, so per-call detection is the right model.
- **Detector instance is built once in `reconfigure`** (`apriltag.Detector(families=...)`) and reused — instantiating per-call would be expensive.
- **Decode and re-encode via cv2.** Source JPEG bytes → `cv2.imdecode` → grayscale for detection → `cv2.polylines` + `cv2.putText` on the BGR image → `cv2.imencode(".jpg", ...)`. No PIL intermediate.
- **Non-JPEG images pass through unchanged.** Depth (e.g. `image/vnd.viam.dep`) and other mime types are returned as-is from `get_images()`.
- **`get_properties` and `get_point_cloud` proxy to the source camera.** This means downstream consumers see the source's intrinsics (important for pose-estimating clients), and depth queries behave normally on cameras that support them.

## Notes from initial bring-up

Verified end-to-end against a RealSense camera at 5 Hz; tags render correctly in the 3D scene.

- **`EasyResource.new` does NOT call `reconfigure` for service models.** Default impl is `cls(config.name); return self`. Component models appear to be auto-reconfigured by the framework, services are not. Our `new` therefore calls `instance.reconfigure(config, dependencies)` explicitly. Without this, the resource serves with all config attributes unset and the detect loop never starts. This was the cause of the initial "module loads but does nothing" failure mode.
- **`validate_config` must return `Tuple[Sequence[str], Sequence[str]]`** (required deps, optional deps) on the current Python SDK. Returning a bare `Sequence[str]` produces a runtime warning `Your validate function validate_config did not return type tuple[Sequence[str], Sequence[str]]` and the second list is treated as empty.
- **`asyncio.create_task` inside sync `reconfigure` works fine** — the SDK runs reconfigure inside an active event loop.
- **`Pose(o_z=1.0)` as geometry-local center renders correctly.** No need to set `theta` explicitly.
- **The 3D scene viewer ignores `Geometry.center`.** The geometry is always drawn at the frame's `pose_in_observer_frame.pose`. Pallet-config hit this too and works around it the same way. If you need a frame origin and a geometry to live at different physical positions, emit two transforms with different reference frames.
- **The renderer caches UUIDs across REMOVED.** Even when we emit `REMOVED(uuid)` immediately followed by `ADDED(uuid)`, the renderer drops the second event and the geometry vanishes. The only fix that produces real-time motion is fresh UUIDs every cycle (we use an epoch-ms suffix). Confirmed by walking through three approaches: stable UUIDs + UPDATED (frozen geometries), stable UUIDs + REMOVED+ADDED (geometries disappear after first cycle), versioned UUIDs (works). Pallet-config arrived at the same conclusion.
- **dt_apriltags uses Y-down tag-local coordinates.** The BL corner is at `(-w/2, +w/2, 0)` in tag-local space, not `(-w/2, -w/2, 0)`. The Z axis returned by the detector points out of the printed surface toward the camera.
- **RealSense reports color intrinsics with depth-as-origin convention.** `viam-camera-realsense` returns the *color* stream's intrinsics (`fx`, `fy`, `ppx`, `ppy`) when `main_sensor` is color, but the camera's frame origin is the depth left imager — see [realsense.hpp](https://github.com/viamrobotics/viam-camera-realsense/blob/main/src/module/realsense.hpp). Pose estimation done with the color intrinsics returns positions in the *color sensor frame*, which is offset ~15 mm in X from the depth imager. The realsense module helpfully populates `extrinsic_parameters.translation` (in mm) with the color→depth offset; we read it each cycle and add it to every emitted tag pose. Auto-no-op for cameras that don't populate the field.
- **RealSense distortion is currently zero.** D-series factory units ship without per-device distortion calibration on the color stream — `Coeffs: [0, 0, 0, 0, 0]`. The realsense module also has distortion publishing commented out (RSDK-12408, unblocked by RDK#5569 as of Dec 2025 but not yet re-enabled in the realsense module). We expose `last_distortion_model` / `last_distortion_params` via `do_command` for diagnosis; once the module re-enables publishing we could wire `cv2.undistort` into the detect loop, but on tested D435/D435i units the coefficients are zeros and there'd be nothing to correct.
- **The 3D scene tab takes a few seconds to subscribe** after the module reconfigures. If `subscriber_count` stays at 0 in `do_command` output, give the renderer time to connect or refresh the page.
- **`viam-sdk` is intentionally unpinned in `requirements.txt`.** If a future install resolves an SDK predating `viam.services.worldstatestore`, the module will ImportError. Pin if and when this bites.

## Don't

- **Don't write images to disk per detection cycle.** A 5 Hz loop would generate 600 files/minute. Logging or capture should be opt-in, not on by default.
- **Don't add config knobs speculatively.** Movement-threshold and disappearance-debounce parameters were considered and intentionally omitted. Add them only when concrete behavior justifies them.
- **Don't retrofit a `PoseTracker` interface onto this model.** If both poll and continuous flavors are wanted, add a second model in this repo sharing the detector wrapper.
- **Don't replace `dt-apriltags`** without a strong reason. It's the easiest path to stay pure-Python and avoid CGO.
