"""PX4 float32 wxyz ingress; reject malformed input before bounded normalization."""
from dataclasses import dataclass
from math import fsum, isfinite, sqrt

# Retain the stricter existing replay limit; never widen it to admit a trace.
# Four rounded float32 components of a unit vector have squared-norm error
# <= 2*u + u*u, u=2**-24 (normal values), well below this ingress ceiling.
# This is a representation/normalization check, NOT an orientation error bound.
MAX_SQUARED_NORM_ERROR = 1e-6


@dataclass(frozen=True)
class NormalizedQuaternion:
    wxyz: tuple[float, float, float, float]
    squared_norm_error: float


def normalize_px4_quaternion(value) -> NormalizedQuaternion:
    q = tuple(float(component) for component in value)
    if len(q) != 4 or not all(isfinite(component) for component in q):
        raise ValueError("INVALID_QUATERNION")
    squared = fsum(component * component for component in q)
    error = squared - 1.0
    if not isfinite(squared) or squared <= 0 or abs(error) > MAX_SQUARED_NORM_ERROR:
        raise ValueError("INVALID_QUATERNION_NORM")
    scale = sqrt(squared)
    return NormalizedQuaternion((q[0]/scale, q[1]/scale, q[2]/scale, q[3]/scale), error)
