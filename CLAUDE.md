# CLAUDE.md — apriltag-tracker

Operational context for future agents working on this repo. Read alongside `README.md` (user-facing).

## What this is

A Viam module that implements two AprilTag-related models:

- `shrews-testing:apriltag-tracker:april_tag_visualizer` (API `rdk:service:world_state_store`) — runs a continuous AprilTag detection loop against a configured camera and publishes each detected tag as a `Transform` so it renders in the Viam app's 3D scene tab.
- `shrews-testing:apriltag-tracker:overlay_camera` (API `rdk:component:camera`) — wraps a source camera and returns annotated JPEGs with each detected tag's four corners drawn as a polygon (rotated/skewed to match the actual tag in the image) and id labelled at the tag center. Provides the 2D companion view to the 3D visualizer.

A separate module, [`viam-labs/apriltag`](https://github.com/viam-labs/apriltag), exposes the same detection capability via the `PoseTracker` component — clients poll `get_poses` to retrieve current detections. This module is the continuous-push counterpart: a background loop computes diffs against the previous cycle and broadcasts `ADDED`/`UPDATED`/`REMOVED` events to subscribers.

## File layout

```
src/main.py          # Module entrypoint. Imports model classes so they self-register, then runs the registry.
src/visualizer.py    # AprilTagVisualizer — the WorldStateStore implementation (3D scene).
src/overlay_camera.py # OverlayCamera — Camera implementation that returns annotated JPEGs (2D scene).
src/spatialmath.py   # quaternion -> Viam orientation vector via libviam_rust_utils ctypes.
libviam_rust_utils-linux_*.so   # Native helpers for the orientation-vector conversion.
meta.json            # Module metadata. Two model entries.
requirements.txt     # Python deps. viam-sdk must be a version that exposes viam.services.worldstatestore.
Makefile             # `make module.tar.gz` packages the module for upload.
run.sh               # viam-server entrypoint. Creates venv, installs deps, exec's `python -m src.main`.
```

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
5. Diffs `new_state` against `self._detected` under `_lock` and broadcasts `ADDED`/`UPDATED`/`REMOVED` events to all subscriber queues.
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

UUIDs are stable across detections; movement is communicated via `UPDATED` events. Each detected tag emits **two** UUIDs per cycle (see "Per-tag transforms" below), so a tag with id `7` adds entries `april_tag_7` and `april_tag_7_origin` to `_detected`.

This diverges from the pattern used by `pallet-webapp-configure-test/pallet-config`, which versions UUIDs (`box-N-v3` → `box-N-v4`) because the Viam 3D renderer was observed to ignore `ADDED` events for cached UUIDs. The pallet-config workaround predates use of `UPDATED` and may be more conservative than necessary. The expectation here is that `UPDATED` works correctly. **If tags appear in the scene but freeze in their initial pose when they move, this is the first thing to suspect.** The fix is to switch movement to REMOVED+ADDED with a version counter (`Dict[tag_id -> int]`) appended to each UUID.

### Per-tag transforms

`_build_transforms(tag)` returns a list of two `Transform` protos for each detected tag:

1. `april_tag_<id>_origin` — a 1mm marker cube placed at the tag's **bottom-left corner**. Its frame has X right, Y up, Z into the tag. This is what carries the user-visible axes triad.
2. `april_tag_<id>` — a `tag_width_mm × tag_width_mm × 1mm` box at the **tag center**. This is the geometry that visually represents the tag's printed face.

Two transforms are required because **the 3D scene viewer ignores `Geometry.center`**: the box is always drawn at the frame's `pose_in_observer_frame.pose` regardless of the offset specified inside the geometry. The pallet-config module hits the same constraint and uses the same workaround — set `pose_in_observer_frame.pose` to where the geometry should land and don't bother with `Geometry.center`. To get both a BL-anchored origin marker AND a tag-area geometry from a single detection, the two anchors must live on separate frames, hence two transforms.

#### Pose math

The detector returns `pose_R` (3x3) and `pose_t` (3x1, in meters because we pass `tag_size = 0.001 * tag_width_mm`).

- **dt_apriltags uses Y-down tag-local coordinates** (image-coordinate convention), so the BL corner is at local `(-w/2, +w/2, 0)`, not `(-w/2, -w/2, 0)`. Getting this wrong puts the origin at the top-left.
- **`t_corner = t + R · (-w/2, +w/2, 0)`** — the BL corner expressed in camera frame.
- **`R_display = R · Rx180`** where `Rx180 = diag(1, -1, -1)`, rotating 180° around X. With the Y-down apriltag frame this yields a display frame with X right, Y up, Z into the tag (which is what the user requested for "Z away from camera").

Both transforms share the same `R_display` orientation and `pose_in_observer_frame.reference_frame = camera.name`. Only the translation differs (BL corner vs. tag center).

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
- **dt_apriltags uses Y-down tag-local coordinates.** The BL corner is at `(-w/2, +w/2, 0)` in tag-local space, not `(-w/2, -w/2, 0)`. The Z axis returned by the detector points out of the printed surface toward the camera.
- **The 3D scene tab takes a few seconds to subscribe** after the module reconfigures. If `subscriber_count` stays at 0 in `do_command` output, give the renderer time to connect or refresh the page.
- **`viam-sdk` is intentionally unpinned in `requirements.txt`.** If a future install resolves an SDK predating `viam.services.worldstatestore`, the module will ImportError. Pin if and when this bites.

## Don't

- **Don't write images to disk per detection cycle.** A 5 Hz loop would generate 600 files/minute. Logging or capture should be opt-in, not on by default.
- **Don't add config knobs speculatively.** Movement-threshold and disappearance-debounce parameters were considered and intentionally omitted. Add them only when concrete behavior justifies them.
- **Don't retrofit a `PoseTracker` interface onto this model.** If both poll and continuous flavors are wanted, add a second model in this repo sharing the detector wrapper.
- **Don't replace `dt-apriltags`** without a strong reason. It's the easiest path to stay pure-Python and avoid CGO.
