import asyncio

from viam.module.module import Module

from .overlay_camera import OverlayCamera  # noqa: F401  (registers the model)
from .visualizer import AprilTagVisualizer  # noqa: F401  (registers the model)


if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
