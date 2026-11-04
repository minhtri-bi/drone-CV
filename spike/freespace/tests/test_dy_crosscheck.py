"""Signed independent residual algebra; no truth-fitted alignment or invented budgets."""

from dataclasses import replace

import numpy as np
import pytest

from dy_crosscheck import DyErrorBudget, independent_dy


# Exact analytic test equations only, never a proposed physical acquisition budget.
EXACT = DyErrorBudget(.5, 0., 0., 0., 0., 0., 0., "Exact analytic plane equations in unit tests")


def evaluate(normal=(2**-.5, 2**-.5, 0.), residuals=(0., 0., 0.), *,
             corrections=(0., 0., 0.), budget: DyErrorBudget | None = EXACT, truth: float | None = 0.):
    return independent_dy(normal, *residuals, nuisance_corrections_m=corrections,
                          budget=budget, truth_minus_estimate_y_m=truth)


def test_oblique_dy_eliminates_dx_dz_and_preserves_signed_truth_minus_estimate():
    normal = np.array([1., 2., 3.])/np.sqrt(14.)
    delta = np.array([.03, -.07, .02])
    # e=-n dot delta; original front and floor normals are +X and +Z.
    errors = (-float(normal @ delta), -delta[0], -delta[2])
    result = evaluate(normal, errors, truth=-.07)
    assert result["dy_estimate_m"] == pytest.approx(-.07, abs=1e-15)
    assert not result["truth_used_for_estimate"] and not result["g2_acceptance"]
    # Floating arithmetic is not hidden behind an invented physical floor budget.
    assert result["dy_error_bound_m"] == 0.


def test_truth_changes_only_comparison_never_estimate():
    first = evaluate(residuals=(-.02, -.01, 0.), truth=0.)
    second = evaluate(residuals=(-.02, -.01, 0.), truth=100.)
    assert first["dy_estimate_m"] == second["dy_estimate_m"]
    assert first["dy_minus_truth_m"] != second["dy_minus_truth_m"]
    assert not second["comparison_pass"]


def test_rotation_depth_extrinsic_corrections_have_explicit_signed_equation():
    normal = np.array([2**-.5, 2**-.5, 0.])
    delta = np.array([.04, -.02, .01])
    nuisance = np.array([.003, -.004, .005])
    measured = -np.array([normal @ delta, delta[0], delta[2]])+nuisance
    corrected = evaluate(normal, measured, corrections=nuisance, truth=-.02)
    uncorrected = evaluate(normal, measured, truth=-.02)
    assert corrected["dy_estimate_m"] == pytest.approx(-.02, abs=1e-15)
    assert abs(uncorrected["dy_estimate_m"]+.02) > .001


def test_exact_y_ambiguity_and_missing_budget_are_explicit_nonacceptance():
    absent = evaluate(budget=None)
    assert absent["failure"] == "MISSING_JUSTIFIED_NUMERICAL_DY_BUDGET"
    assert absent["dy_estimate_m"] == 0. and not absent["comparison_pass"]
    ambiguous = evaluate((1., 0., 0.), budget=None)
    assert ambiguous["failure"] == "Y_TRANSLATION_UNIDENTIFIABLE"
    assert ambiguous["dy_estimate_m"] is None
    assert not ambiguous["identifiable"]


def test_small_y_normal_is_not_accepted_with_large_noise_amplification():
    normal = (np.sqrt(1-.01**2), .01, 0.)
    result = evaluate(normal)
    assert not result["identifiable"] and result["failure"] == "INSUFFICIENT_Y_NORMAL_CONDITIONING"
    assert not result["comparison_pass"]


def test_numerator_budget_weights_front_floor_and_nuisance_before_dividing():
    budget = DyErrorBudget(.5, .01, .02, .03, .04, .05, .06, "Analytic boundary test only")
    normal = (.6, .8, 0.)
    result = evaluate(normal, budget=budget, truth=0.)
    expected = (.01+.6*.02+.04+.05+.06)/.8
    assert result["dy_error_bound_m"] == pytest.approx(expected)
    assert result["comparison_pass"] and not result["motion_authority"]
    boundary = evaluate(normal, budget=budget, truth=expected)
    assert boundary["comparison_pass"]
    outside = evaluate(normal, budget=budget, truth=expected+1e-10)
    assert not outside["comparison_pass"]
    missing = evaluate(normal, budget=budget, truth=None)
    assert missing["failure"] == "SAME_BAG_TRUTH_COMPARISON_MISSING"


@pytest.mark.parametrize("changes", [{"minimum_abs_normal_y": 0.}, {"depth_numerator_m": np.nan},
                                      {"rotation_numerator_m": -.1}, {"provenance": ""}])
def test_budget_never_supplies_silent_defaults_or_invalid_numbers(changes):
    with pytest.raises(ValueError, match="INVALID_DY_ERROR_BUDGET"):
        replace(EXACT, **changes)


@pytest.mark.parametrize("normal,residuals,corrections", [
    ((1., 1., 0.), (0., 0., 0.), (0., 0., 0.)),
    ((0., 1., 0.), (np.nan, 0., 0.), (0., 0., 0.)),
    ((0., 1., 0.), (0., 0., 0.), (0., np.inf, 0.)),
])
def test_nonfinite_and_nonunit_inputs_reject(normal, residuals, corrections):
    with pytest.raises(ValueError, match="INVALID_DY_RESIDUAL_INPUT"):
        evaluate(normal, residuals, corrections=corrections)


def test_auxiliary_normal_sign_change_preserves_translation_estimate():
    normal = (2**-.5, 2**-.5, 0.)
    original = evaluate(normal, (-.03, -.01, 0.))
    reversed_normal = tuple(-component for component in normal)
    reversed_result = evaluate(reversed_normal, (.03, -.01, 0.))
    assert original["dy_estimate_m"] == reversed_result["dy_estimate_m"]


def test_unbounded_numeric_y_amplification_never_emits_infinite_estimate():
    result = evaluate((1., 1e-320, 0.), (-1., 0., 0.), budget=None)
    assert result["failure"] == "Y_TRANSLATION_NUMERICALLY_UNBOUNDED"
    assert result["dy_estimate_m"] is None
    assert not result["comparison_pass"]
