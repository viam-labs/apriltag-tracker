import asyncio

from viam.module.module import Module

from .visualizer import AprilTagVisualizer  # noqa: F401  (registers the model)


if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
