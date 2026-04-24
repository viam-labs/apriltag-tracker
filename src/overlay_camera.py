from typing import Any, ClassVar, List, Mapping, Optional, Sequence, Tuple, cast

import cv2
import dt_apriltags as apriltag
import numpy as np
from typing_extensions import Self

from viam.components.camera import Camera
from viam.logging import getLogger
from viam.media.video import CameraMimeType, NamedImage, ViamImage
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, ResponseMetadata
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

LOGGER = getLogger(__name__)

CAMERA_ATTR = "camera_name"
FAMILY_ATTR = "tag_family"

POLY_COLOR = (0, 255, 0)  # BGR — green
TEXT_COLOR = (0, 0, 255)  # BGR — red
POLY_THICKNESS = 2
FONT_SCALE = 0.8
TEXT_THICKNESS = 2


class OverlayCamera(Camera, EasyResource):
    MODEL: ClassVar[Model] = Model(
        ModelFamily("shrews-testing", "apriltag-tracker"), "overlay_camera"
    )

    def __init__(self, name: str):
        super().__init__(name)
        self._detector: Optional[apriltag.Detector] = None
        self.source_camera: Optional[Camera] = None
        self.tag_family: Optional[str] = None

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
        return [str(cam)], []

    def reconfigure(
        self,
        config: ComponentConfig,
        dependencies: Mapping[ResourceName, ResourceBase],
    ):
        attrs = struct_to_dict(config.attributes)
        self.source_camera = cast(
            Camera,
            dependencies.get(Camera.get_resource_name(str(attrs[CAMERA_ATTR]))),
        )
        self.tag_family = str(attrs[FAMILY_ATTR])
        self._detector = apriltag.Detector(families=self.tag_family)
        LOGGER.info(
            f"overlay_camera reconfigure: source={self.source_camera.name} "
            f"family={self.tag_family}"
        )

    async def get_image(
        self,
        mime_type: str = "",
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> ViamImage:
        requested = mime_type or CameraMimeType.JPEG
        src = await self.source_camera.get_image(mime_type=requested)
        if src.mime_type == CameraMimeType.JPEG:
            return ViamImage(
                data=self._annotate_jpeg(src.data),
                mime_type=CameraMimeType.JPEG,
            )
        return src

    async def get_images(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[List[NamedImage], ResponseMetadata]:
        src_images, metadata = await self.source_camera.get_images()
        out: List[NamedImage] = []
        for img in src_images:
            if img.mime_type == CameraMimeType.JPEG:
                out.append(
                    NamedImage(
                        name=img.name,
                        data=self._annotate_jpeg(img.data),
                        mime_type=CameraMimeType.JPEG,
                    )
                )
            else:
                out.append(img)
        return out, metadata

    async def get_properties(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Camera.Properties:
        return await self.source_camera.get_properties()

    async def get_point_cloud(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bytes, str]:
        return await self.source_camera.get_point_cloud()

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        raise NotImplementedError()

    def _annotate_jpeg(self, jpeg_bytes: bytes) -> bytes:
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return jpeg_bytes
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        tags = self._detector.detect(gray)
        for tag in tags:
            corners = tag.corners.astype(int).reshape(-1, 1, 2)
            cv2.polylines(
                bgr, [corners], isClosed=True, color=POLY_COLOR, thickness=POLY_THICKNESS
            )
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv2.putText(
                bgr,
                str(tag.tag_id),
                (cx, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                FONT_SCALE,
                TEXT_COLOR,
                TEXT_THICKNESS,
            )
        ok, encoded = cv2.imencode(".jpg", bgr)
        if not ok:
            return jpeg_bytes
        return encoded.tobytes()
