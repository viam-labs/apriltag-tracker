import asyncio
import math
import time
from typing import Any, AsyncGenerator, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple, cast

import cv2
import dt_apriltags as apriltag
import numpy as np
from scipy.spatial.transform import Rotation
from typing_extensions import Self

from viam.components.camera import Camera
from viam.logging import getLogger
from viam.media.utils.pil import viam_to_pil_image
from viam.media.video import CameraMimeType
from viam.module.module import Module  # noqa: F401
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import (
    Geometry,
    Pose,
    PoseInFrame,
    RectangularPrism,
    ResourceName,
    Transform,
    Vector3,
)
from viam.proto.service.worldstatestore import (
    StreamTransformChangesResponse,
    TransformChangeType,
)
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.worldstatestore import WorldStateStore
from viam.utils import ValueTypes, struct_to_dict

from .spatialmath import quaternion_to_orientation_vector

LOGGER = getLogger(__name__)

CAMERA_ATTR = "camera_name"
FAMILY_ATTR = "tag_family"
WIDTH_ATTR = "tag_width_mm"
RATE_ATTR = "detection_rate_hz"
DEFAULT_RATE_HZ = 5.0


class AprilTagVisualizer(WorldStateStore, EasyResource):
    MODEL: ClassVar[Model] = Model(
        ModelFamily("shrews-testing", "apriltag-tracker"), "april_tag_visualizer"
    )

    def __init__(self, name: str):
        super().__init__(name)
        self._lock = asyncio.Lock()
        self._detected: Dict[bytes, Transform] = {}
        self._subscribers: List[asyncio.Queue] = []
        self._loop_task: Optional[asyncio.Task] = None
        # Debug state — exposed via do_command.
        self._loop_started_at: Optional[float] = None
        self._cycles_completed: int = 0
        self._last_cycle_at: Optional[float] = None
        self._last_cycle_error: Optional[str] = None
        self._last_intrinsics: List[float] = []
        self._last_distortion_model: Optional[str] = None
        self._last_distortion_params: List[float] = []
        # Color-sensor origin offset in the camera's reference frame (mm).
        # RealSense reports color-stream intrinsics but treats the depth
        # left imager as the camera frame origin; this offset compensates.
        self._sensor_offset_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_image_mime_types: List[str] = []
        self._last_gray_shape: Optional[Tuple[int, int]] = None
        self._last_tag_ids: List[int] = []

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        instance = super().new(config, dependencies)
        instance.reconfigure(config, dependencies)
        return instance

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        cam = attrs.get(CAMERA_ATTR)
        if cam is None:
            raise Exception(f"Missing required {CAMERA_ATTR} attribute.")
        if attrs.get(FAMILY_ATTR) is None:
            raise Exception(f"Missing required {FAMILY_ATTR} attribute.")
        if attrs.get(WIDTH_ATTR) is None:
            raise Exception(f"Missing required {WIDTH_ATTR} attribute.")
        rate = attrs.get(RATE_ATTR, DEFAULT_RATE_HZ)
        if float(rate) <= 0:
            raise Exception(f"{RATE_ATTR} must be > 0.")
        return [str(cam)], []

    def reconfigure(
        self,
        config: ComponentConfig,
        dependencies: Mapping[ResourceName, ResourceBase],
    ):
        attrs = struct_to_dict(config.attributes)
        self.camera = cast(
            Camera,
            dependencies.get(Camera.get_resource_name(str(attrs[CAMERA_ATTR]))),
        )
        self.tag_family = str(attrs[FAMILY_ATTR])
        self.tag_width_mm = float(attrs[WIDTH_ATTR])
        self.detection_rate_hz = float(attrs.get(RATE_ATTR, DEFAULT_RATE_HZ))

        if self._loop_task is not None:
            self._loop_task.cancel()
        self._detected = {}
        self._cycles_completed = 0
        self._last_cycle_error = None
        LOGGER.info(
            f"reconfigure: camera={self.camera.name} family={self.tag_family} "
            f"tag_width_mm={self.tag_width_mm} rate_hz={self.detection_rate_hz}"
        )
        self._loop_task = asyncio.create_task(self._detect_loop())

    async def close(self):
        if self._loop_task is not None:
            self._loop_task.cancel()
            try:
                await self._loop_task
            except asyncio.CancelledError:
                pass

    async def _detect_loop(self):
        self._loop_started_at = time.time()
        period = 1.0 / self.detection_rate_hz
        try:
            detector = apriltag.Detector(families=self.tag_family)
        except Exception as e:
            self._last_cycle_error = f"detector init failed: {type(e).__name__}: {e}"
            LOGGER.error(self._last_cycle_error)
            raise
        LOGGER.info(f"detection loop started at {self.detection_rate_hz} Hz")
        while True:
            try:
                await self._detect_once(detector)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._last_cycle_error = f"{type(e).__name__}: {e}"
                LOGGER.warning(f"detection cycle failed: {self._last_cycle_error}")
            await asyncio.sleep(period)

    async def _detect_once(self, detector):
        properties = await self.camera.get_properties()
        intrinsics = [
            properties.intrinsic_parameters.focal_x_px,
            properties.intrinsic_parameters.focal_y_px,
            properties.intrinsic_parameters.center_x_px,
            properties.intrinsic_parameters.center_y_px,
        ]
        self._last_intrinsics = list(intrinsics)
        self._last_distortion_model = properties.distortion_parameters.model
        self._last_distortion_params = list(properties.distortion_parameters.parameters)
        self._sensor_offset_mm = (
            properties.extrinsic_parameters.translation.x,
            properties.extrinsic_parameters.translation.y,
            properties.extrinsic_parameters.translation.z,
        )

        cam_images = await self.camera.get_images()
        self._last_image_mime_types = [img.mime_type for img in cam_images[0]]
        gray = None
        for image in cam_images[0]:
            if image.mime_type == CameraMimeType.JPEG:
                gray = cv2.cvtColor(
                    np.array(viam_to_pil_image(image)), cv2.COLOR_RGB2GRAY
                )
                break
        if gray is None:
            self._last_gray_shape = None
            self._last_cycle_error = (
                f"no JPEG image found; mime types: {self._last_image_mime_types}"
            )
            return
        self._last_gray_shape = (gray.shape[0], gray.shape[1])

        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=intrinsics,
            tag_size=0.001 * self.tag_width_mm,
        )
        self._last_tag_ids = [int(t.tag_id) for t in tags]
        self._last_cycle_at = time.time()
        self._cycles_completed += 1
        # Clear last error on a successful cycle.
        self._last_cycle_error = None

        new_state: Dict[bytes, Transform] = {}
        for tag in tags:
            for tf in self._build_transforms(tag):
                new_state[tf.uuid] = tf

        async with self._lock:
            # The 3D scene renderer doesn't redraw on UPDATED events for
            # cached UUIDs, so we tear everything down and re-add. REMOVED
            # for the previous cycle's entries comes first to clear the
            # renderer's cache, then ADDED for the new cycle.
            for t in self._detected.values():
                self._broadcast(
                    StreamTransformChangesResponse(
                        change_type=TransformChangeType.TRANSFORM_CHANGE_TYPE_REMOVED,
                        transform=t,
                    )
                )
            for t in new_state.values():
                self._broadcast(
                    StreamTransformChangesResponse(
                        change_type=TransformChangeType.TRANSFORM_CHANGE_TYPE_ADDED,
                        transform=t,
                    )
                )
            self._detected = new_state

    def _build_transforms(self, tag) -> List[Transform]:
        """Two transforms per tag: a BL-corner origin frame plus a
        tag-center frame carrying the box geometry. Two transforms are
        needed because the renderer ignores Geometry.center, so the
        box is always drawn at the frame's pose — to have both a BL
        origin marker and a geometry covering the tag area, the two
        anchors must live on separate frames."""
        w_mm = self.tag_width_mm
        half_m = w_mm / 2.0 / 1000.0  # meters; pose_t is in meters

        R = tag.pose_R
        t = tag.pose_t.flatten()

        # dt_apriltags uses Y-down tag-local coords, so BL is at
        # (-w/2, +w/2, 0) — not (-w/2, -w/2, 0) which is top-left.
        t_corner = t + R @ np.array([-half_m, half_m, 0.0])

        # Flip Z by rotating 180° around X (keeps X, flips Y and Z).
        # Combined with the Y-down apriltag convention this yields a
        # display frame with X right, Y up, Z into the tag.
        Rx180 = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
        R_display = R @ Rx180
        o = quaternion_to_orientation_vector(Rotation.from_matrix(R_display))

        # Translate from the reported intrinsics' sensor frame (color)
        # into the camera's reference frame (depth left imager).
        ox, oy, oz = self._sensor_offset_mm

        pose_corner = Pose(
            x=t_corner[0] * 1000 + ox,
            y=t_corner[1] * 1000 + oy,
            z=t_corner[2] * 1000 + oz,
            o_x=o.o_x,
            o_y=o.o_y,
            o_z=o.o_z,
            theta=o.theta * 180 / math.pi,
        )
        pose_center = Pose(
            x=t[0] * 1000 + ox,
            y=t[1] * 1000 + oy,
            z=t[2] * 1000 + oz,
            o_x=o.o_x,
            o_y=o.o_y,
            o_z=o.o_z,
            theta=o.theta * 180 / math.pi,
        )

        origin_name = f"april_tag_{tag.tag_id}_origin"
        box_name = f"april_tag_{tag.tag_id}"

        # Origin marker: 10mm cube at the BL corner so the renderer has
        # something visible to draw axes against. 1mm fell below the
        # renderer's "annotate this frame" size threshold.
        origin_tf = Transform(
            uuid=origin_name.encode(),
            reference_frame=origin_name,
            pose_in_observer_frame=PoseInFrame(
                reference_frame=self.camera.name, pose=pose_corner
            ),
            physical_object=Geometry(
                box=RectangularPrism(dims_mm=Vector3(x=10.0, y=10.0, z=10.0)),
                label=origin_name,
            ),
        )
        # Tag area: full-size box at the tag center.
        box_tf = Transform(
            uuid=box_name.encode(),
            reference_frame=box_name,
            pose_in_observer_frame=PoseInFrame(
                reference_frame=self.camera.name, pose=pose_center
            ),
            physical_object=Geometry(
                box=RectangularPrism(dims_mm=Vector3(x=w_mm, y=w_mm, z=1.0)),
                label=box_name,
            ),
        )
        return [origin_tf, box_tf]

    def _broadcast(self, msg: StreamTransformChangesResponse):
        for q in list(self._subscribers):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                LOGGER.warning("subscriber queue full; dropping event")

    async def list_uuids(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> List[bytes]:
        async with self._lock:
            return list(self._detected.keys())

    async def get_transform(
        self,
        uuid: bytes,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> Transform:
        async with self._lock:
            t = self._detected.get(uuid)
            if t is None:
                raise Exception(f"unknown uuid {uuid!r}")
            return t

    async def stream_transform_changes(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
    ) -> AsyncGenerator[StreamTransformChangesResponse, None]:
        q: asyncio.Queue = asyncio.Queue(maxsize=256)
        async with self._lock:
            self._subscribers.append(q)
            for t in self._detected.values():
                q.put_nowait(
                    StreamTransformChangesResponse(
                        change_type=TransformChangeType.TRANSFORM_CHANGE_TYPE_ADDED,
                        transform=t,
                    )
                )
        try:
            while True:
                yield await q.get()
        finally:
            async with self._lock:
                if q in self._subscribers:
                    self._subscribers.remove(q)

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        loop_running = self._loop_task is not None and not self._loop_task.done()
        loop_exception = None
        if self._loop_task is not None and self._loop_task.done():
            try:
                exc = self._loop_task.exception()
                if exc is not None:
                    loop_exception = f"{type(exc).__name__}: {exc}"
            except (asyncio.CancelledError, asyncio.InvalidStateError):
                pass

        return {
            "loop_running": loop_running,
            "loop_started_at": self._loop_started_at,
            "loop_exception": loop_exception,
            "cycles_completed": self._cycles_completed,
            "last_cycle_at": self._last_cycle_at,
            "last_cycle_error": self._last_cycle_error,
            "last_intrinsics": self._last_intrinsics,
            "last_distortion_model": self._last_distortion_model,
            "last_distortion_params": self._last_distortion_params,
            "sensor_offset_mm": list(self._sensor_offset_mm),
            "last_image_mime_types": self._last_image_mime_types,
            "last_gray_shape": list(self._last_gray_shape) if self._last_gray_shape else None,
            "last_tag_ids": self._last_tag_ids,
            "current_tracked_count": len(self._detected),
            "current_tracked_uuids": [u.decode() for u in self._detected.keys()],
            "subscriber_count": len(self._subscribers),
            "config": {
                "camera_name": self.camera.name if getattr(self, "camera", None) else None,
                "tag_family": getattr(self, "tag_family", None),
                "tag_width_mm": getattr(self, "tag_width_mm", None),
                "detection_rate_hz": getattr(self, "detection_rate_hz", None),
            },
        }
