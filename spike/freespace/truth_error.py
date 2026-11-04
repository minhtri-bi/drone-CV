"""Evaluator-only full-SE(3) errors; measured stationary samples are not calibration."""

from bisect import bisect_left, bisect_right
from math import atan2, sin

import numpy as np

from capture_pose import quaternion_rotation


def quaternion(matrix):
    """Unit wxyz from a proper rotation, including rotations near pi."""
    rotation = np.asarray(matrix, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or abs(np.linalg.det(rotation)-1.) > 1e-8):
        raise ValueError("INVALID_TRUTH_ROTATION")
    trace = np.trace(rotation)
    if trace > 0:
        scale = 2*np.sqrt(trace+1)
        result = np.array([scale/4, (rotation[2, 1]-rotation[1, 2])/scale,
                           (rotation[0, 2]-rotation[2, 0])/scale,
                           (rotation[1, 0]-rotation[0, 1])/scale])
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i+1) % 3, (i+2) % 3
        scale = 2*np.sqrt(max(0., 1+rotation[i, i]-rotation[j, j]-rotation[k, k]))
        xyz = np.zeros(3)
        xyz[i] = scale/4
        xyz[j] = (rotation[j, i]+rotation[i, j])/scale
        xyz[k] = (rotation[k, i]+rotation[i, k])/scale
        result = np.r_[(rotation[k, j]-rotation[j, k])/scale, xyz]
    return result/np.linalg.norm(result)


def rigid(pose):
    result = np.eye(4)
    result[:3, :3] = pose.world_from_body()
    result[:3, 3] = pose.position
    return result


def inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def interpolate_truth(samples, stamp, max_gap_ns):
    """Bracket only, no extrapolation; caller supplies one validated clock epoch."""
    if type(max_gap_ns) is not int or max_gap_ns <= 0:
        raise ValueError("INVALID_TRUTH_GAP")
    stamps = [sample[0] for sample in samples]
    if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("NONMONOTONIC_TRUTH")
    index = bisect_left(stamps, stamp)
    if index < len(stamps) and stamps[index] == stamp:
        return np.asarray(samples[index][1]), [0, 0]
    if index == 0 or index == len(stamps):
        raise ValueError("UNBRACKETED_TRUTH")
    left_stamp, left = samples[index-1]
    right_stamp, right = samples[index]
    if right_stamp-left_stamp > max_gap_ns:
        raise ValueError("TRUTH_GAP")
    fraction = (stamp-left_stamp)/(right_stamp-left_stamp)
    q0, q1 = quaternion(left[:3, :3]), quaternion(right[:3, :3])
    dot = float(q0 @ q1)
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > .9995:
        q = q0+fraction*(q1-q0)
        q /= np.linalg.norm(q)
    else:
        angle = np.arccos(np.clip(dot, -1., 1.))
        q = (sin((1-fraction)*angle)*q0+sin(fraction*angle)*q1)/sin(angle)
    result = np.eye(4)
    result[:3, :3] = quaternion_rotation(q)
    result[:3, 3] = (1-fraction)*left[:3, 3]+fraction*right[:3, 3]
    return result, [left_stamp-stamp, right_stamp-stamp]


def ray_radius(camera, optical_z_m):
    """Full outer pixel-cone corners; optical Z is not Euclidean ray range."""
    if not np.isfinite(optical_z_m) or not 0 < optical_z_m <= camera.far_m:
        raise ValueError("INVALID_DIAGNOSTIC_Z_CAP")
    k = camera.intrinsics
    rays = np.array([[(u-k.cx)/k.fx, (v-k.cy)/k.fy, 1.]
                     for u in (-.5, k.width-.5) for v in (-.5, k.height-.5)])
    far_points = rays*optical_z_m
    lever = np.asarray(camera.mount.translation_flu)
    body_from_optical = camera.mount.body_from_optical()
    body_points = far_points @ body_from_optical.T+lever
    range_max = float(np.linalg.norm(far_points, axis=1).max())
    return {"optical_z_cap_m": optical_z_m, "maximum_ray_range_m": range_max,
            "lever_arm_m": float(np.linalg.norm(lever)),
            "rho_m": max(float(np.linalg.norm(body_points, axis=1).max()), float(np.linalg.norm(lever))),
            "triangle_rho_bound_m": range_max+float(np.linalg.norm(lever)),
            "intrinsics": {"width": k.width, "height": k.height, "fx": k.fx, "fy": k.fy,
                           "cx": k.cx, "cy": k.cy},
            "lever_arm_body_m": lever.tolist(), "body_from_optical": body_from_optical.tolist(),
            "pixel_boundaries": [-.5, k.width-.5, -.5, k.height-.5]}


def transform_error(error, rho):
    q = quaternion(error[:3, :3])
    angle = 2*atan2(float(np.linalg.norm(q[1:])), abs(float(q[0])))
    translation = float(np.linalg.norm(error[:3, 3]))
    return {"translation_m": translation, "rotation_rad": angle,
            "bound_m": translation+2*sin(angle/2)*rho,
            "linear_bound_m": translation+angle*rho}


def relative_error(estimated_capture, estimated_query, truth_capture, truth_query):
    """E=(That_q^-1 That_c)^-1 (T_q^-1 T_c), in the capture-body basis."""
    estimated = inverse(estimated_query) @ estimated_capture
    truth = inverse(truth_query) @ truth_capture
    return inverse(estimated) @ truth


def all_pairs(captures, queries, ttl_ns, rho):
    """Every observed capture and query pair within TTL, not adjacent frames only.

    Each row is (source_ns, estimated_world_from_body, true_world_from_body).
    Queries are the actual recorded PX4 sample times, not a continuous-time claim.
    """
    if type(ttl_ns) is not int or ttl_ns <= 0 or not np.isfinite(rho) or rho <= 0:
        raise ValueError("INVALID_TTL_OR_RADIUS")
    for rows in (captures, queries):
        if not rows or any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
            raise ValueError("NONMONOTONIC_ERROR_SAMPLES")
    worst = {"maximum_translation_m": 0., "maximum_rotation_rad": 0.,
             "maximum_bound_m": 0., "maximum_linear_bound_m": 0.}
    worst_pairs = {}
    count, maximum_age_ns = 0, 0
    capture_stamps = [row[0] for row in captures]
    covered_query_count = 0
    for query_ns, estimated_query, truth_query in queries:
        first = bisect_left(capture_stamps, query_ns-ttl_ns)
        stop = bisect_right(capture_stamps, query_ns)
        covered_query_count += stop > first
        for index in range(first, stop):
            capture_ns, estimated_capture, truth_capture = captures[index]
            age = query_ns-capture_ns
            error = relative_error(estimated_capture, estimated_query, truth_capture, truth_query)
            measured = transform_error(error, rho)
            count += 1
            maximum_age_ns = max(maximum_age_ns, age)
            for field, value in measured.items():
                key = "maximum_"+field
                if value >= worst[key]:
                    worst[key] = value
                    worst_pairs[key] = {"capture_ns": capture_ns, "query_ns": query_ns,
                                        "age_ns": age, **measured, "error_transform": error.tolist()}
    if not count:
        raise ValueError("NO_OBSERVED_TTL_PAIRS")
    return {**worst, "pair_count": count, "query_count": len(queries),
            "queries_with_retained_capture": covered_query_count,
            "capture_count": len(captures), "maximum_observed_age_ns": maximum_age_ns,
            "ttl_ns": ttl_ns, "rho_m": rho, "worst_pairs": worst_pairs,
            "query_domain": "all recorded PX4 timestamp_sample values with truth brackets",
            "continuous_time_bound_certified": False}
