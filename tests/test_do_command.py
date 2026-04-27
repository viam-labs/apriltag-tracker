"""Async tests for the do_command dispatch surface."""
from types import SimpleNamespace

import numpy as np
import pytest

from src.visualizer import AprilTagVisualizer


def _bare_visualizer():
    v = AprilTagVisualizer.__new__(AprilTagVisualizer)
    AprilTagVisualizer.__init__(v, "test")
    v.camera = SimpleNamespace(name="cam")
    v.tag_family = "tag36h11"
    v.tag_width_mm = 150.0
    v.detection_rate_hz = 5.0
    v.centroid_alpha = 1.0
    return v


def _populate_two_tags(v, ts_ms=1234567890):
    tag_a = SimpleNamespace(
        tag_id=21, pose_R=np.eye(3), pose_t=np.array([0.0, 0.0, 1.0]).reshape(3, 1)
    )
    tag_b = SimpleNamespace(
        tag_id=42, pose_R=np.eye(3), pose_t=np.array([0.5, 0.0, 1.5]).reshape(3, 1)
    )
    new_state = {}
    for tag in (tag_a, tag_b):
        for tf in v._build_transforms(tag, ts_ms):
            new_state[tf.uuid] = tf
    v._detected = new_state
    v._cycle_ts = ts_ms


# ---------- list_tags ----------

@pytest.mark.asyncio
async def test_list_tags_returns_sorted_unique_ids():
    v = _bare_visualizer()
    _populate_two_tags(v)
    result = await v.do_command({"command": "list_tags"})
    assert result["tags"] == [21, 42]
    assert result["timestamp_ms"] == 1234567890


@pytest.mark.asyncio
async def test_list_tags_empty_when_nothing_detected():
    v = _bare_visualizer()
    result = await v.do_command({"command": "list_tags"})
    assert result["tags"] == []


# ---------- list_uuids ----------

@pytest.mark.asyncio
async def test_list_uuids_returns_string_uuids():
    v = _bare_visualizer()
    _populate_two_tags(v, ts_ms=999)
    result = await v.do_command({"command": "list_uuids"})
    uuids = set(result["uuids"])
    assert uuids == {
        "april_tag_21_origin_999",
        "april_tag_21_centroid_999",
        "april_tag_42_origin_999",
        "april_tag_42_centroid_999",
    }


# ---------- get_pose ----------

@pytest.mark.asyncio
async def test_get_pose_returns_origin_and_centroid_for_known_id():
    v = _bare_visualizer()
    _populate_two_tags(v)
    result = await v.do_command({"command": "get_pose", "tag_id": 21})
    assert result["tag_id"] == 21
    # Both nested entries exist with camera_frame populated; world_frame
    # is None here because the bare visualizer has no motion service set.
    assert result["origin"]["camera_frame"] is not None
    assert result["origin"]["world_frame"] is None
    assert result["centroid"]["camera_frame"] is not None
    assert result["centroid"]["world_frame"] is None
    assert result["camera_name"] == "cam"


@pytest.mark.asyncio
async def test_get_pose_returns_null_for_unknown_id():
    v = _bare_visualizer()
    _populate_two_tags(v)
    result = await v.do_command({"command": "get_pose", "tag_id": 999})
    assert result == {"tag_id": 999, "origin": None, "centroid": None}


@pytest.mark.asyncio
async def test_get_pose_accepts_string_tag_id():
    v = _bare_visualizer()
    _populate_two_tags(v)
    result = await v.do_command({"command": "get_pose", "tag_id": "21"})
    assert result["tag_id"] == 21
    assert result["centroid"]["camera_frame"] is not None


@pytest.mark.asyncio
async def test_get_pose_missing_tag_id_raises():
    v = _bare_visualizer()
    with pytest.raises(Exception, match="tag_id"):
        await v.do_command({"command": "get_pose"})


@pytest.mark.asyncio
async def test_get_pose_world_frame_uses_motion_service():
    """When a motion service is set, world_frame should be populated
    by composing through motion.get_pose."""
    v = _bare_visualizer()
    _populate_two_tags(v)

    # Stub motion: returns the camera-frame pose offset by (1000, 0, 0)
    # in world, regardless of input. Just enough to verify wiring.
    from viam.proto.common import Pose as ProtoPose, PoseInFrame

    class StubMotion:
        async def get_pose(self, *, component_name, destination_frame, supplemental_transforms):
            assert destination_frame == "world"
            assert len(supplemental_transforms) == 1
            sup = supplemental_transforms[0]
            assert sup.reference_frame == component_name
            base = sup.pose_in_observer_frame.pose
            return PoseInFrame(
                reference_frame="world",
                pose=ProtoPose(
                    x=base.x + 1000, y=base.y, z=base.z,
                    o_x=base.o_x, o_y=base.o_y, o_z=base.o_z,
                    theta=base.theta,
                ),
            )

    v.motion = StubMotion()
    result = await v.do_command({"command": "get_pose", "tag_id": 21})
    assert result["centroid"]["world_frame"] is not None
    assert result["centroid"]["world_frame"]["x"] == pytest.approx(
        result["centroid"]["camera_frame"]["x"] + 1000
    )
    assert result["origin"]["world_frame"] is not None


# ---------- get_transforms ----------

@pytest.mark.asyncio
async def test_get_transforms_returns_each_uuid():
    v = _bare_visualizer()
    _populate_two_tags(v)
    result = await v.do_command({"command": "get_transforms"})
    assert len(result["transforms"]) == 4
    labels = {entry["label"] for entry in result["transforms"]}
    assert labels == {
        "april_tag_21_origin", "april_tag_21_centroid",
        "april_tag_42_origin", "april_tag_42_centroid",
    }


@pytest.mark.asyncio
async def test_get_transforms_includes_metadata_when_alpha_set():
    v = _bare_visualizer()
    v.centroid_alpha = 0.3
    _populate_two_tags(v)
    result = await v.do_command({"command": "get_transforms"})
    centroid_entries = [
        e for e in result["transforms"] if "centroid" in e["label"]
    ]
    assert len(centroid_entries) == 2
    for entry in centroid_entries:
        assert entry["metadata"] == {"opacity": pytest.approx(0.3)}


# ---------- default fall-through ----------

@pytest.mark.asyncio
async def test_no_command_returns_debug_snapshot():
    v = _bare_visualizer()
    result = await v.do_command({})
    # Spot-check a few debug fields.
    assert "loop_running" in result
    assert "cycles_completed" in result
    assert "config" in result
    assert result["config"]["centroid_alpha"] == 1.0
