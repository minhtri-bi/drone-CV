"""Pure read-only runtime context checks shared by discovery and output ticks."""
from drone_cv.px4_observation import (
    BatteryObservation, FallbackObservation, HomeObservation,
    LocalObservation, observation_failure,
)
from drone_cv.px4_policy import (
    PolicyGeometry, conservative_upper_float_parameter_bound, validate_geometry_policy,
)
from drone_cv.supervisor import Config, Stage, State, stopping_distance
from drone_cv.timing import Clock, freshness_error


def runtime_context_error(latest: dict, now: Clock, cfg: Config,
                          geometry: PolicyGeometry, policy: dict, state: State) -> str | None:
    """Agreement is conditional, never proof that fallback or landing is safe.

    Home is static and frozen by reference/reset/epoch. Required dynamic sources
    keep their original acquisition and receive clocks; no refreshed cache age.
    """
    expected = (("home", HomeObservation), ("local", LocalObservation),
                ("flags", FallbackObservation), ("battery", BatteryObservation))
    for name, kind in expected:
        if not isinstance(latest.get(name), kind):
            return "MISSING_" + name.upper() + "_CONTEXT"
    home, local, flags, battery = (latest[name] for name, _ in expected)
    for name, value in (("LOCAL_POSITION", local), ("FALLBACK_FLAGS", flags),
                        ("BATTERY", battery), ("ODOMETRY", latest.get("odometry")),
                        ("STATUS", latest.get("status"))):
        if value is None:
            return "MISSING_" + name
        error = freshness_error(value.stamp, now, cfg.source(name).freshness(cfg.clock_contract))
        if error:
            return name + "_" + error
    # A static Home does not need synthetic source-timestamp progress, but its
    # source may not belong to a different epoch or a future clock.
    if (home.stamp.epoch != now.epoch or home.stamp.sim_s > now.sim_s
            or home.stamp.received_wall_s > now.wall_s):
        return "HOME_CLOCK_SCOPE"
    offboard_required = state.stage in (Stage.OFFBOARD_DISARMED, Stage.ARM_WAIT, Stage.ACTIVE)
    for value in (local, flags, battery):
        error = observation_failure(value, offboard_required=offboard_required)
        if error:
            return error
    if battery.remaining <= conservative_upper_float_parameter_bound("BAT_LOW_THR", policy["BAT_LOW_THR"]):
        return "BATTERY_RESERVE_UNHEALTHY"
    actual, status = latest["odometry"], latest["status"]
    if state.stage == Stage.IDLE and (status.arming_state != 1 or status.nav_state == 14):
        return "STARTUP_AUTHORITY_NOT_AVAILABLE"
    if status.failsafe or status.executor_in_charge != 0:
        return "PX4_FALLBACK_ALREADY_ACTIVE"
    env = cfg.envelope
    if any(value > ceiling for vector, ceilings in (
            (actual.position_variance, env.position_variance_max),
            (actual.velocity_variance, env.velocity_variance_max))
           for value, ceiling in zip(vector, ceilings)):
        return "ODOMETRY_UNCERTAINTY"
    if env.capture_query_bound is not None and (
            env.capture_query_bound.clock_epoch != now.epoch
            or env.capture_query_bound.reset_counter != actual.reset_counter):
        return "RELATIVE_UNCERTAINTY_RUNTIME_SCOPE"
    try:
        validate_geometry_policy(policy, geometry, current_home=home.bind(local),
            minimum_ned=env.minimum_ned, maximum_ned=env.maximum_ned,
            required_motion_reserve_m=stopping_distance(cfg, actual.velocity_ned,
                                                       state.issued_speed_bound_mps),
            body_provenance=env.bounds_provenance)
    except (ValueError, TypeError) as error:
        return "PX4_GEOMETRY:" + str(error)
    return None
