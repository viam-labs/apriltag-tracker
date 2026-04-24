import asyncio
import math
from typing import Any, AsyncGenerator, ClassVar, Dict, List, Mapping, Optional, Sequence, cast

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

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        return super().new(config, dependencies)

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
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
        return [str(cam)]

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
        self._loop_task = asyncio.create_task(self._detect_loop())

    async def close(self):
        if self._loop_task is not None:
            self._loop_task.cancel()
            try:
                await self._loop_task
            except asyncio.CancelledError:
                pass

    async def _detect_loop(self):
        period = 1.0 / self.detection_rate_hz
        detector = apriltag.Detector(families=self.tag_family)
        while True:
            try:
                await self._detect_once(detector)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                LOGGER.warning(f"detection cycle failed: {e}")
            await asyncio.sleep(period)

    async def _detect_once(self, detector):
        properties = await self.camera.get_properties()
        intrinsics = [
            properties.intrinsic_parameters.focal_x_px,
            properties.intrinsic_parameters.focal_y_px,
            properties.intrinsic_parameters.center_x_px,
            properties.intrinsic_parameters.center_y_px,
        ]
        cam_images = await self.camera.get_images()
        gray = None
        for image in cam_images[0]:
            if image.mime_type == CameraMimeType.JPEG:
                gray = cv2.cvtColor(
                    np.array(viam_to_pil_image(image)), cv2.COLOR_RGB2GRAY
                )
                break
        if gray is None:
            return

        tags = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=intrinsics,
            tag_size=0.001 * self.tag_width_mm,
        )

        new_state: Dict[bytes, Transform] = {}
        for tag in tags:
            uuid = str(tag.tag_id).encode()
            new_state[uuid] = self._build_transform(tag, uuid)

        async with self._lock:
            old_keys = set(self._detected.keys())
            new_keys = set(new_state.keys())

            for k in new_keys - old_keys:
                self._broadcast(
                    StreamTransformChangesResponse(
                        change_type=TransformChangeType.TRANSFORM_CHANGE_TYPE_ADDED,
                        transform=new_state[k],
                    )
                )
            for k in new_keys & old_keys:
                self._broadcast(
                    StreamTransformChangesResponse(
                        change_type=TransformChangeType.TRANSFORM_CHANGE_TYPE_UPDATED,
                        transform=new_state[k],
                    )
                )
            for k in old_keys - new_keys:
                self._broadcast(
                    StreamTransformChangesResponse(
                        change_type=TransformChangeType.TRANSFORM_CHANGE_TYPE_REMOVED,
                        transform=self._detected[k],
                    )
                )
            self._detected = new_state

    def _build_transform(self, tag, uuid: bytes) -> Transform:
        o = quaternion_to_orientation_vector(Rotation.from_matrix(tag.pose_R))
        pose = Pose(
            x=tag.pose_t[0][0] * 1000,
            y=tag.pose_t[1][0] * 1000,
            z=tag.pose_t[2][0] * 1000,
            o_x=o.o_x,
            o_y=o.o_y,
            o_z=o.o_z,
            theta=o.theta * 180 / math.pi,
        )
        geometry = Geometry(
            center=Pose(o_z=1.0),
            box=RectangularPrism(
                dims_mm=Vector3(x=self.tag_width_mm, y=self.tag_width_mm, z=1.0)
            ),
            label=f"tag-{tag.tag_id}",
        )
        return Transform(
            uuid=uuid,
            reference_frame=f"tag-{tag.tag_id}",
            pose_in_observer_frame=PoseInFrame(
                reference_frame=self.camera.name, pose=pose
            ),
            physical_object=geometry,
        )

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
        raise NotImplementedError()
