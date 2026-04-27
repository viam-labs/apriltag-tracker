"""Unit tests for src/visualizer.py.

Run from the repo root: ``.venv/bin/pytest tests/`` (the spatialmath
helper loads libviam_rust_utils via a relative path, so cwd matters).
"""
from types import SimpleNamespace

import numpy as np
import pytest
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Pose
from viam.utils import dict_to_struct, struct_to_dict

from src.visualizer import (
    AprilTagVisualizer,
    DEFAULT_CENTROID_ALPHA,
    _pose_to_dict,
)


# ---------- helpers ----------

def _make_config(attrs):
    return ComponentConfig(attributes=dict_to_struct(attrs))


def _bare_visualizer(camera_name="cam", tag_width_mm=150.0, centroid_alpha=1.0):
    """Construct a visualizer instance bypassing `new()` so tests don't
    need a live camera dependency. Sets the attributes that
    `_build_transforms` and `do_command` read."""
    v = AprilTagVisualizer.__new__(AprilTagVisualizer)
    AprilTagVisualizer.__init__(v, "test_visualizer")
    v.camera = SimpleNamespace(name=camera_name)
    v.tag_family = "tag36h11"
    v.tag_width_mm = tag_width_mm
    v.detection_rate_hz = 5.0
    v.centroid_alpha = centroid_alpha
    return v


def _fake_tag(tag_id=21, t=(0.0, 0.0, 1.0), R=None):
    """Mimic the fields of a `dt_apriltags.Detection` that
    `_build_transforms` reads."""
    if R is None:
        R = np.eye(3)
    return SimpleNamespace(
        tag_id=tag_id, pose_R=R, pose_t=np.array(t).reshape(3, 1)
    )


# ---------- _pose_to_dict ----------

def test_pose_to_dict_returns_all_fields():
    pose = Pose(x=1, y=2, z=3, o_x=0.1, o_y=0.2, o_z=0.3, theta=45)
    assert _pose_to_dict(pose) == {
        "x": 1, "y": 2, "z": 3,
        "o_x": 0.1, "o_y": 0.2, "o_z": 0.3,
        "theta": 45,
    }


# ---------- validate_config ----------

def test_validate_config_returns_required_and_optional_deps():
    cfg = _make_config({
        "camera_name": "realsense-cam",
        "tag_family": "tag36h11",
        "tag_width_mm": 150.0,
    })
    required, optional = AprilTagVisualizer.validate_config(cfg)
    assert required == ["realsense-cam"]
    # Default motion service name is "builtin", declared optional so the
    # framework provides it when present and we degrade gracefully when not.
    assert optional == ["builtin"]


def test_validate_config_uses_configured_motion_service_name():
    cfg = _make_config({
        "camera_name": "c", "tag_family": "tag36h11",
        "tag_width_mm": 150.0, "motion_service_name": "my-motion",
    })
    required, optional = AprilTagVisualizer.validate_config(cfg)
    assert optional == ["my-motion"]


def test_validate_config_missing_camera_name_raises():
    cfg = _make_config({"tag_family": "tag36h11", "tag_width_mm": 150.0})
    with pytest.raises(Exception, match="camera_name"):
        AprilTagVisualizer.validate_config(cfg)


def test_validate_config_missing_tag_family_raises():
    cfg = _make_config({"camera_name": "c", "tag_width_mm": 150.0})
    with pytest.raises(Exception, match="tag_family"):
        AprilTagVisualizer.validate_config(cfg)


def test_validate_config_missing_tag_width_raises():
    cfg = _make_config({"camera_name": "c", "tag_family": "tag36h11"})
    with pytest.raises(Exception, match="tag_width_mm"):
        AprilTagVisualizer.validate_config(cfg)


def test_validate_config_rejects_non_positive_rate():
    cfg = _make_config({
        "camera_name": "c", "tag_family": "tag36h11",
        "tag_width_mm": 150.0, "detection_rate_hz": 0,
    })
    with pytest.raises(Exception, match="detection_rate_hz"):
        AprilTagVisualizer.validate_config(cfg)


def test_validate_config_rejects_alpha_out_of_range():
    cfg = _make_config({
        "camera_name": "c", "tag_family": "tag36h11",
        "tag_width_mm": 150.0, "centroid_alpha": 1.5,
    })
    with pytest.raises(Exception, match="centroid_alpha"):
        AprilTagVisualizer.validate_config(cfg)


def test_validate_config_accepts_alpha_zero_and_one():
    for alpha in (0.0, 1.0, 0.5):
        cfg = _make_config({
            "camera_name": "c", "tag_family": "tag36h11",
            "tag_width_mm": 150.0, "centroid_alpha": alpha,
        })
        AprilTagVisualizer.validate_config(cfg)  # should not raise


# ---------- _build_transforms ----------

def test_build_transforms_emits_two_transforms_per_tag():
    v = _bare_visualizer(tag_width_mm=150.0)
    tag = _fake_tag(tag_id=42, t=(0.0, 0.0, 1.0))
    transforms = v._build_transforms(tag, ts_ms=1234567890)
    assert len(transforms) == 2


def test_build_transforms_uuids_carry_timestamp_suffix():
    v = _bare_visualizer()
    tag = _fake_tag(tag_id=7)
    origin, centroid = v._build_transforms(tag, ts_ms=999)
    assert origin.uuid == b"april_tag_7_origin_999"
    assert centroid.uuid == b"april_tag_7_centroid_999"
    assert origin.reference_frame == "april_tag_7_origin_999"
    assert centroid.reference_frame == "april_tag_7_centroid_999"


def test_build_transforms_labels_are_unsuffixed():
    v = _bare_visualizer()
    tag = _fake_tag(tag_id=7)
    origin, centroid = v._build_transforms(tag, ts_ms=999)
    assert origin.physical_object.label == "april_tag_7_origin"
    assert centroid.physical_object.label == "april_tag_7_centroid"


def test_build_transforms_observer_frame_is_camera_name():
    v = _bare_visualizer(camera_name="my-cam")
    tag = _fake_tag()
    origin, centroid = v._build_transforms(tag, ts_ms=1)
    assert origin.pose_in_observer_frame.reference_frame == "my-cam"
    assert centroid.pose_in_observer_frame.reference_frame == "my-cam"


def test_build_transforms_geometry_dims():
    v = _bare_visualizer(tag_width_mm=150.0)
    tag = _fake_tag()
    origin, centroid = v._build_transforms(tag, ts_ms=1)
    # Origin marker is a 10mm cube.
    assert origin.physical_object.box.dims_mm.x == 10.0
    assert origin.physical_object.box.dims_mm.y == 10.0
    assert origin.physical_object.box.dims_mm.z == 10.0
    # Centroid is tag_width_mm × tag_width_mm × 1mm.
    assert centroid.physical_object.box.dims_mm.x == 150.0
    assert centroid.physical_object.box.dims_mm.y == 150.0
    assert centroid.physical_object.box.dims_mm.z == 1.0


def test_build_transforms_bl_corner_offset_with_identity_rotation():
    """With R=I and tag center at (0,0,1), the BL corner should sit
    at (-w/2, +w/2, 0) relative to the center because the apriltag
    tag-local Y axis points down (image-coordinate convention)."""
    v = _bare_visualizer(tag_width_mm=200.0)
    tag = _fake_tag(tag_id=5, t=(0.0, 0.0, 1.0), R=np.eye(3))
    origin, centroid = v._build_transforms(tag, ts_ms=1)
    # Centroid sits at the detector-reported translation (in mm).
    assert centroid.pose_in_observer_frame.pose.x == pytest.approx(0.0)
    assert centroid.pose_in_observer_frame.pose.y == pytest.approx(0.0)
    assert centroid.pose_in_observer_frame.pose.z == pytest.approx(1000.0)
    # BL corner is offset by (-100, +100, 0) mm from the center.
    assert origin.pose_in_observer_frame.pose.x == pytest.approx(-100.0)
    assert origin.pose_in_observer_frame.pose.y == pytest.approx(100.0)
    assert origin.pose_in_observer_frame.pose.z == pytest.approx(1000.0)


def test_build_transforms_applies_sensor_offset():
    """sensor_offset_mm should be added to both translations after the
    meters→mm conversion, in the camera reference frame."""
    v = _bare_visualizer(tag_width_mm=200.0)
    v._sensor_offset_mm = (14.65, 0.18, 0.34)
    tag = _fake_tag(tag_id=5, t=(0.0, 0.0, 1.0), R=np.eye(3))
    origin, centroid = v._build_transforms(tag, ts_ms=1)
    # Centroid: (0, 0, 1000) + (14.65, 0.18, 0.34)
    assert centroid.pose_in_observer_frame.pose.x == pytest.approx(14.65)
    assert centroid.pose_in_observer_frame.pose.y == pytest.approx(0.18)
    assert centroid.pose_in_observer_frame.pose.z == pytest.approx(1000.34)
    # Origin: (-100, 100, 1000) + (14.65, 0.18, 0.34)
    assert origin.pose_in_observer_frame.pose.x == pytest.approx(-85.35)
    assert origin.pose_in_observer_frame.pose.y == pytest.approx(100.18)
    assert origin.pose_in_observer_frame.pose.z == pytest.approx(1000.34)


def test_build_transforms_sets_opacity_metadata_when_alpha_lt_one():
    v = _bare_visualizer(centroid_alpha=0.4)
    tag = _fake_tag()
    _origin, centroid = v._build_transforms(tag, ts_ms=1)
    metadata = struct_to_dict(centroid.metadata)
    assert metadata == {"opacity": pytest.approx(0.4)}


def test_build_transforms_no_metadata_when_alpha_is_one():
    v = _bare_visualizer(centroid_alpha=1.0)
    tag = _fake_tag()
    _origin, centroid = v._build_transforms(tag, ts_ms=1)
    metadata = struct_to_dict(centroid.metadata)
    assert metadata == {}


def test_build_transforms_marker_metadata_always_empty():
    """Origin marker should never carry opacity metadata, even when
    centroid_alpha is set."""
    v = _bare_visualizer(centroid_alpha=0.1)
    tag = _fake_tag()
    origin, _centroid = v._build_transforms(tag, ts_ms=1)
    metadata = struct_to_dict(origin.metadata)
    assert metadata == {}


# ---------- _tag_ids_from_detected ----------

def test_tag_ids_from_detected_extracts_unique_ids():
    v = _bare_visualizer()
    # Construct fake transforms only using the bits _tag_ids_from_detected reads.
    def fake(label):
        return SimpleNamespace(physical_object=SimpleNamespace(label=label))
    v._detected = {
        b"k1": fake("april_tag_5_centroid"),
        b"k2": fake("april_tag_5_origin"),
        b"k3": fake("april_tag_12_centroid"),
        b"k4": fake("april_tag_12_origin"),
        b"k5": fake("not_an_apriltag_label"),
    }
    assert v._tag_ids_from_detected() == {5, 12}


def test_tag_ids_from_detected_empty_when_nothing_tracked():
    v = _bare_visualizer()
    v._detected = {}
    assert v._tag_ids_from_detected() == set()


# ---------- defaults ----------

def test_default_centroid_alpha_is_opaque():
    assert DEFAULT_CENTROID_ALPHA == 1.0
