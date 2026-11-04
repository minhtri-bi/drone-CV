from math import pi
from types import SimpleNamespace

import numpy as np
import pytest

from drone_cv.oracle_profile import _primitive_points, rotor_bounds
from drone_cv.oracle_truth import WIRE_LABEL, body_truth, wire_bytes


def test_rotor_sweep_uses_joint_axis_not_translated_mesh_origin():
    points = np.array([[3., 4., -1.], [1., 0., 2.]])
    lo, hi, radius = rotor_bounds(points, np.array([1., 0., 0.]), np.array([0., 0., 1.]))
    assert radius == pytest.approx(np.sqrt(20))
    assert lo == pytest.approx([1-np.sqrt(20), -np.sqrt(20), -1])
    assert hi == pytest.approx([1+np.sqrt(20), np.sqrt(20), 2])


def test_rotor_sweep_encloses_rotation_about_a_tilted_axis():
    axis = np.array([0., 1., 1.])/np.sqrt(2)
    hub = np.array([.4, -.2, .1])
    points = np.array([[.7, .5, .2], [-.3, .2, -.5]])
    lo, hi, _ = rotor_bounds(points, hub, axis)
    for angle in np.linspace(0, 2*pi, 31):
        offsets = points-hub
        rotated = (offsets*np.cos(angle)+np.cross(axis, offsets)*np.sin(angle)
                   +(offsets @ axis)[:, None]*axis*(1-np.cos(angle)))+hub
        assert np.all(rotated >= lo-1e-12)
        assert np.all(rotated <= hi+1e-12)


def test_visual_plane_preserves_width_and_height_axes():
    vector = SimpleNamespace(x=lambda: 0., y=lambda: 0., z=lambda: 1.)
    size = SimpleNamespace(x=lambda: 4., y=lambda: 2.)
    plane = SimpleNamespace(normal=lambda: vector, size=lambda: size)
    geometry = SimpleNamespace(box_shape=lambda: None, plane_shape=lambda: plane)
    points = _primitive_points(geometry)
    assert points.min(axis=0) == pytest.approx([-2., -1., 0.])
    assert points.max(axis=0) == pytest.approx([2., 1., 0.])


def pose(name, identity, xyz):
    return SimpleNamespace(name=name, id=identity,
                           position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
                           orientation=SimpleNamespace(w=1., x=0., y=0., z=0.))


def truth_message(poses):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=5, nsec=20)), pose=poses)


ENTITIES = {"model_name": "x500_depth_0", "model_id": 18, "body_name": "base_link",
            "body_id": 19, "link_pose_convention": "model_relative"}


def test_truth_composes_named_model_and_link_not_array_order():
    result = body_truth(truth_message([pose("base_link", 19, [0, 0, .24]),
                                      pose("x500_depth_0", 18, [1, 2, -.013])]), ENTITIES)
    assert np.asarray(result.world_from_body)[:3, 3] == pytest.approx([1, 2, .227])
    assert result.source_ns == 5_000_000_020


@pytest.mark.parametrize("poses", [
    [pose("base_link", 19, [0, 0, .24])],
    [pose("x500_depth_0", 18, [0, 0, 0]), pose("base_link", 20, [0, 0, .24])],
    [pose("x500_depth_0", 18, [0, 0, 0]), pose("base_link", 19, [0, 0, .24]),
     pose("base_link", 20, [0, 0, .24])],
])
def test_truth_rejects_missing_changed_or_ambiguous_entities(poses):
    with pytest.raises(ValueError, match="IDENTITY_CHANGED"):
        body_truth(truth_message(poses), ENTITIES)


def test_truth_wire_never_accepts_truncated_declared_payload():
    message = SimpleNamespace(data=[1, 2], layout=SimpleNamespace(data_offset=0,
        dim=[SimpleNamespace(label=WIRE_LABEL, size=3, stride=3)]))
    with pytest.raises(ValueError, match="WIRE_LAYOUT"):
        wire_bytes(message)
