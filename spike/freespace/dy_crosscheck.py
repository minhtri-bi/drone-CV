"""Independent evaluator-only dy from plane residuals, with explicit budget failures."""

from dataclasses import asdict, dataclass
from math import isfinite
from numbers import Real

import numpy as np


@dataclass(frozen=True)
class DyErrorBudget:
    """Declared input error bounds, never a calibration certificate.

    The three residual bounds cover plane location/fit/ROI estimation. Rotation,
    depth and extrinsic terms bound the *combined numerator* after front/floor
    elimination, not three unweighted independent plane errors. Their numerical
    derivation and source must be frozen before collection, not fitted to truth.
    """

    minimum_abs_normal_y: float
    auxiliary_residual_m: float
    front_residual_m: float
    floor_residual_m: float
    rotation_numerator_m: float
    depth_numerator_m: float
    extrinsic_numerator_m: float
    provenance: str

    def __post_init__(self):
        values = (self.minimum_abs_normal_y, self.auxiliary_residual_m,
                  self.front_residual_m, self.floor_residual_m, self.rotation_numerator_m,
                  self.depth_numerator_m, self.extrinsic_numerator_m)
        if (not all(type(v) in (float, int) and isfinite(v) and v >= 0 for v in values)
                or not 0 < self.minimum_abs_normal_y <= 1
                or not isinstance(self.provenance, str) or not self.provenance.strip()):
            raise ValueError("INVALID_DY_ERROR_BUDGET")


def independent_dy(normal, auxiliary_residual_m, front_residual_m, floor_residual_m, *,
                   nuisance_corrections_m, budget: DyErrorBudget | None,
                   truth_minus_estimate_y_m: float | None):
    """Estimate delta=truth translation minus estimate, without fitting alignment.

    For each measured point, e = n dot (p_est - plane_point) = -n dot delta + nu.
    nu includes rotation about the body origin with the camera lever arm, optical
    depth error, and extrinsic error. With explicitly supplied corrections,
    r = -e + nu = n_x*dx + n_y*dy + n_z*dz. Original front and floor normals
    are +X and +Z, so dx=r_front and dz=r_floor. No truth value participates
    in the estimate; optional same-bag truth is used only for its final residual.
    Corrections are ordered (auxiliary, front, floor), with signs along each
    plane's declared normal. Explicit zero means uncorrected diagnostic, not an
    assertion that nuisance error is physically zero.
    """
    normal = np.asarray(normal, dtype=float)
    corrections = np.asarray(nuisance_corrections_m, dtype=float)
    residuals = (auxiliary_residual_m, front_residual_m, floor_residual_m)
    if (normal.shape != (3,) or not np.isfinite(normal).all()
            or not np.isclose(np.linalg.norm(normal), 1., atol=1e-10, rtol=0)
            or corrections.shape != (3,) or not np.isfinite(corrections).all()
            or any(not isinstance(v, Real) or isinstance(v, bool) or not isfinite(v) for v in residuals)
            or (truth_minus_estimate_y_m is not None and not isfinite(truth_minus_estimate_y_m))):
        raise ValueError("INVALID_DY_RESIDUAL_INPUT")
    if budget is not None and not isinstance(budget, DyErrorBudget):
        raise ValueError("EXPLICIT_DY_ERROR_BUDGET_REQUIRED")
    nx, ny, nz = (float(v) for v in normal)
    result = {"scope": "INDEPENDENT_DY_DIAGNOSTIC_NOT_CALIBRATION", "motion_authority": False,
              "g2_acceptance": False, "normal_world": normal.tolist(),
              "signed_plane_residuals_m": dict(zip(("auxiliary", "front", "floor"), residuals)),
              "signed_nuisance_corrections_m": corrections.tolist(),
              "equation": "r=-e+nu; r_aux=nx*dx+ny*dy+nz*dz; dx=r_front; dz=r_floor",
              "truth_used_for_estimate": False, "identifiable": ny != 0.,
              "budget": asdict(budget) if budget is not None else None,
              "dy_estimate_m": None, "truth_minus_estimate_y_m": truth_minus_estimate_y_m,
              "dy_minus_truth_m": None, "dy_error_bound_m": None, "comparison_pass": False}
    if ny == 0.:
        return {**result, "failure": "Y_TRANSLATION_UNIDENTIFIABLE"}
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        corrected = -np.asarray(residuals)+corrections
        dy = float((corrected[0]-nx*corrected[1]-nz*corrected[2])/ny)
    if not isfinite(dy):
        return {**result, "identifiable": False, "failure": "Y_TRANSLATION_NUMERICALLY_UNBOUNDED"}
    result["dy_estimate_m"] = dy
    if truth_minus_estimate_y_m is not None:
        result["dy_minus_truth_m"] = dy-truth_minus_estimate_y_m
        if not isfinite(result["dy_minus_truth_m"]):
            return {**result, "dy_minus_truth_m": None, "failure": "DY_TRUTH_COMPARISON_UNBOUNDED"}
    if budget is None:
        return {**result, "failure": "MISSING_JUSTIFIED_NUMERICAL_DY_BUDGET"}
    if abs(ny) < budget.minimum_abs_normal_y:
        return {**result, "identifiable": False, "failure": "INSUFFICIENT_Y_NORMAL_CONDITIONING"}
    bound = ((budget.auxiliary_residual_m+abs(nx)*budget.front_residual_m
              +abs(nz)*budget.floor_residual_m+budget.rotation_numerator_m
              +budget.depth_numerator_m+budget.extrinsic_numerator_m)/abs(ny))
    if not isfinite(bound):
        return {**result, "failure": "DY_ERROR_BOUND_UNBOUNDED"}
    result["dy_error_bound_m"] = bound
    if truth_minus_estimate_y_m is None:
        return {**result, "failure": "SAME_BAG_TRUTH_COMPARISON_MISSING"}
    passed = abs(result["dy_minus_truth_m"]) <= bound
    return {**result, "comparison_pass": passed,
            "failure": None if passed else "DY_TRUTH_RESIDUAL_EXCEEDS_DECLARED_BUDGET"}
