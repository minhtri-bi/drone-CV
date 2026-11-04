"""Pure, pinned PX4 exploratory-policy compatibility checks.

A successful check is conditional agreement, NOT a measured braking bound,
physical landing certificate, Hold guarantee, or authority to construct output.
No ROS imports, parameter writes, defaults, or runtime acquisition live here.
"""
from dataclasses import dataclass
from math import hypot, isfinite
import struct
from types import MappingProxyType

PINNED_FIRMWARE = "61907962c9220437ec01a98fa26bfbf5cbb254b5"


@dataclass(frozen=True)
class ParameterDomain:
    kind: str
    minimum: float
    maximum: float
    values: tuple[int, ...] = ()


# Source inventory and the distinction between source domains and the narrower
# exploratory policy are documented in docs/PX4_POLICY_CONTRACT.md.
PARAMETER_DOMAINS = MappingProxyType({
    "COM_OF_LOSS_T": ParameterDomain("float", 0, 60),
    "COM_OBL_RC_ACT": ParameterDomain("int32", 0, 7, tuple(range(8))),
    "COM_RC_IN_MODE": ParameterDomain("int32", 0, 8, tuple(range(9))),
    "COM_FAIL_ACT_T": ParameterDomain("float", 0, 25),
    "COM_RC_LOSS_T": ParameterDomain("float", 0, 35),
    "COM_RCL_EXCEPT": ParameterDomain("int32", 0, 31),
    "COM_DLL_EXCEPT": ParameterDomain("int32", 0, 31),
    "NAV_RCL_ACT": ParameterDomain("int32", 1, 7, (1, 2, 3, 5, 6, 7)),
    "NAV_DLL_ACT": ParameterDomain("int32", 0, 6, (0, 1, 2, 3, 5, 6)),
    "COM_DL_LOSS_T": ParameterDomain("int32", 5, 300),
    "COM_LOW_BAT_ACT": ParameterDomain("int32", 0, 4, (0, 2, 3, 4)),
    "BAT_LOW_THR": ParameterDomain("float", .12, .5),
    "BAT_CRIT_THR": ParameterDomain("float", .05, .5),
    "BAT_EMERGEN_THR": ParameterDomain("float", .03, .5),
    "GF_ACTION": ParameterDomain("int32", 0, 5, tuple(range(6))),
    "GF_SOURCE": ParameterDomain("int32", 0, 1, (0, 1)),
    "GF_MAX_HOR_DIST": ParameterDomain("float", 0, 10000),
    "GF_MAX_VER_DIST": ParameterDomain("float", 0, 10000),
    "MPC_XY_VEL_MAX": ParameterDomain("float", 0, 20),
    "MPC_Z_VEL_MAX_UP": ParameterDomain("float", .5, 8),
    "MPC_Z_VEL_MAX_DN": ParameterDomain("float", .5, 4),
    "MPC_POS_MODE": ParameterDomain("int32", 0, 4, (0, 4)),
    "MPC_ACC_HOR": ParameterDomain("float", 2, 15),
    "MPC_ACC_HOR_MAX": ParameterDomain("float", 2, 15),
    "MPC_ACC_UP_MAX": ParameterDomain("float", 2, 15),
    "MPC_ACC_DOWN_MAX": ParameterDomain("float", 2, 15),
    "MPC_JERK_MAX": ParameterDomain("float", .5, 500),
    "MPC_JERK_AUTO": ParameterDomain("float", 1, 80),
    "UXRCE_DDS_SYNCT": ParameterDomain("int32", 0, 1, (0, 1)),
})
REQUIRED_PARAMETERS = frozenset(PARAMETER_DOMAINS)


def _number(value, name: str) -> None:
    if type(value) not in (int, float) or not isfinite(value):
        raise ValueError(f"{name} must be a finite number, not a boolean")


def _positive(value, name: str) -> None:
    _number(value, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive")


def _identity(value, name: str) -> None:
    if not isinstance(value, str) or not value.strip() or len(value) > 512:
        raise ValueError(f"{name} requires explicit bounded provenance/rationale")


def _vector(value, name: str) -> tuple[float, float, float]:
    result = tuple(value)
    if len(result) != 3:
        raise ValueError(f"{name} must be a finite NED 3-vector")
    for component in result:
        _number(component, name)
    return result


def _box(instance) -> None:
    for name in ("minimum_ned", "maximum_ned"):
        object.__setattr__(instance, name, _vector(getattr(instance, name), name))
    if any(lo >= hi for lo, hi in zip(instance.minimum_ned, instance.maximum_ned)):
        raise ValueError("reference-center box must have positive extent on every axis")


def _float32(value: float) -> float:
    try:
        return struct.unpack("!f", struct.pack("!f", value))[0]
    except OverflowError:
        return float("inf") if value > 0 else -float("inf")


def _from_ordered_bits(value: int) -> float:
    bits = value ^ 0x80000000 if value & 0x80000000 else ~value & 0xffffffff
    return struct.unpack("!f", struct.pack("!I", bits))[0]


def float32_comparison_bounds(expected: float) -> tuple[float, float]:
    """Exact finite binary32 endpoints accepted by pinned ``param compare``.

    PX4 parses the argument as double then converts to float, subtracts two
    floats and tests fabsf(diff) < 1e-7f. Ordered-bit binary searches include
    that float subtraction rounding, not Python isclose or quiet-text rounding.
    Endpoints are inclusive; every finite binary32 outside them fails compare.
    """
    _number(expected, "float comparison target")
    target = _float32(expected)
    if not isfinite(target):
        raise ValueError("comparison target must fit finite float32")
    bits = struct.unpack("!I", struct.pack("!f", target))[0]
    center = (~bits & 0xffffffff) if bits & 0x80000000 else bits ^ 0x80000000
    limit = _float32(1e-7)

    def accepted(ordered):
        return abs(_float32(_from_ordered_bits(ordered) - target)) < limit

    # Ordered finite binary32 domain, excluding both infinities and all NaNs.
    left, right = 0x00800000, center
    while left < right:
        middle = (left + right) // 2
        if accepted(middle):
            right = middle
        else:
            left = middle + 1
    lower = _from_ordered_bits(left)
    left, right = center, 0xff7fffff
    while left < right:
        middle = (left + right + 1) // 2
        if accepted(middle):
            left = middle
        else:
            right = middle - 1
    return lower, _from_ordered_bits(left)


def conservative_upper_float_parameter_bound(name: str, expected: float) -> float:
    """Upper live-value bound conditional on a successful pinned comparison.

    This helper does not perform or manufacture readback attestation. Callers
    must validate the full inventory and require real comparison success first.
    """
    if name not in PARAMETER_DOMAINS or PARAMETER_DOMAINS[name].kind != "float":
        raise ValueError("upper-bound helper requires a pinned float parameter")
    return float32_comparison_bounds(expected)[1]


def parameter_precision(name: str, expected: int | float) -> dict:
    domain = PARAMETER_DOMAINS[name]
    if domain.kind == "int32":
        return {"type": "int32", "lower": expected, "upper": expected,
                "comparison": "exact_integer"}
    lower, upper = float32_comparison_bounds(expected)
    return {"type": "float32", "expected_float32": _float32(expected),
            "lower": lower, "upper": upper,
            "comparison_absolute_limit": _float32(1e-7),
            "comparison": "fabsf_float32_difference_strictly_less_than_limit",
            "quiet_decimal_places": 4}


def validate_parameter_policy(expected: dict, rationale: str) -> None:
    """Validate canonical types/full inventory, then restricted action policy.

    A battery Land configuration alone is insufficient: geometry/context and
    current health must also pass before runtime admission.
    """
    if not isinstance(expected, dict) or set(expected) != REQUIRED_PARAMETERS:
        raise ValueError("failsafe policy requires the complete pinned parameter inventory")
    _identity(rationale, "frozen trial policy rationale")
    for name, domain in PARAMETER_DOMAINS.items():
        value = expected[name]
        _number(value, name)
        if domain.kind == "int32" and type(value) is not int:
            raise ValueError(f"{name} requires canonical int32 policy type")
        if domain.kind == "float" and type(value) is not float:
            raise ValueError(f"{name} requires canonical float policy type")
        if not domain.minimum <= value <= domain.maximum or (domain.values and value not in domain.values):
            raise ValueError(f"parameter outside pinned domain: {name}")
    if expected["UXRCE_DDS_SYNCT"] != 0:
        raise ValueError("simulation requires Gazebo clock without DDS wall synchronization")
    for name, value in {"COM_OBL_RC_ACT": 5, "GF_ACTION": 2, "GF_SOURCE": 0,
                        "NAV_RCL_ACT": 1, "NAV_DLL_ACT": 1, "COM_FAIL_ACT_T": 0,
                        "COM_RCL_EXCEPT": 0, "COM_DLL_EXCEPT": 0,
                        "COM_RC_IN_MODE": 4, "COM_LOW_BAT_ACT": 2}.items():
        if expected[name] != value:
            raise ValueError(f"unsupported restricted exploratory failsafe policy: {name}")
    if expected["COM_OF_LOSS_T"] <= 0 or expected["MPC_XY_VEL_MAX"] <= 0:
        raise ValueError("loss timeout and horizontal speed cap must be positive")
    # Fence enabling in the actual source is > FLT_EPSILON, not merely > 0.
    for name in ("GF_MAX_HOR_DIST", "GF_MAX_VER_DIST"):
        if float32_comparison_bounds(expected[name])[0] <= 2**-23:
            raise ValueError("both Home-centered outer fences must be enabled")
    if not (float32_comparison_bounds(expected["BAT_EMERGEN_THR"])[1]
            < float32_comparison_bounds(expected["BAT_CRIT_THR"])[0]
            < float32_comparison_bounds(expected["BAT_CRIT_THR"])[1]
            < float32_comparison_bounds(expected["BAT_LOW_THR"])[0]):
        raise ValueError("battery emergency/critical/low thresholds must remain strictly ordered")


def validate_motion_policy(expected: dict, *, maximum_speed_mps: float,
                           minimum_deceleration_mps2: float) -> None:
    """Reject inconsistent declarations; configured caps NEVER prove response.

    The minimum of axis caps is only an impossibility check for a claimed
    all-direction minimum deceleration. Actual minimum braking, acceleration
    response, jerk onset/delay and applicable workload need independent evidence.
    """
    validate_parameter_policy(expected, "motion compatibility check")
    _positive(maximum_speed_mps, "maximum resultant speed")
    _positive(minimum_deceleration_mps2, "declared minimum deceleration")
    horizontal = conservative_upper_float_parameter_bound("MPC_XY_VEL_MAX", expected["MPC_XY_VEL_MAX"])
    vertical = max(conservative_upper_float_parameter_bound(name, expected[name])
                   for name in ("MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN"))
    if hypot(horizontal, vertical) > maximum_speed_mps:
        raise ValueError("3D resultant configured speed plus comparison reserve exceeds declaration")
    # Hold's autonomous trajectory uses MPC_ACC_HOR. The direct-velocity
    # position variant additionally uses MPC_ACC_HOR_MAX; mode 4 does not.
    caps = ("MPC_ACC_HOR", "MPC_ACC_UP_MAX", "MPC_ACC_DOWN_MAX")
    if expected["MPC_POS_MODE"] == 0:
        caps += ("MPC_ACC_HOR_MAX",)
    if minimum_deceleration_mps2 > min(float32_comparison_bounds(expected[name])[0] for name in caps):
        raise ValueError("declared minimum deceleration exceeds a commanded acceleration upper limit")


@dataclass(frozen=True)
class LocalReference:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    ref_timestamp_us: int

    def __post_init__(self):
        for name in ("latitude_deg", "longitude_deg", "altitude_m"):
            _number(getattr(self, name), name)
        if not -90 <= self.latitude_deg <= 90 or not -180 <= self.longitude_deg <= 180:
            raise ValueError("local reference latitude/longitude outside domain")
        if type(self.ref_timestamp_us) is not int or not 0 < self.ref_timestamp_us < 2**64:
            raise ValueError("local reference requires positive uint64 reference timestamp")


@dataclass(frozen=True)
class HomeSnapshot:
    position_ned: tuple[float, float, float]
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    update_count: int
    local_reference: LocalReference
    epoch: str
    reset_counters: tuple[int, int, int, int]

    def __post_init__(self):
        object.__setattr__(self, "position_ned", _vector(self.position_ned, "Home position"))
        for name in ("latitude_deg", "longitude_deg", "altitude_m"):
            _number(getattr(self, name), name)
        if not -90 <= self.latitude_deg <= 90 or not -180 <= self.longitude_deg <= 180:
            raise ValueError("Home latitude/longitude outside domain")
        if type(self.update_count) is not int or not 0 <= self.update_count < 2**32:
            raise ValueError("Home update count must be uint32")
        if not isinstance(self.local_reference, LocalReference):
            raise ValueError("Home requires an actual local reference binding")
        _identity(self.epoch, "Home epoch")
        counters = tuple(self.reset_counters)
        if len(counters) != 4 or any(type(v) is not int or not 0 <= v <= 255 for v in counters):
            raise ValueError("Home reset provenance requires xy/z/vxy/vz uint8 counters")
        object.__setattr__(self, "reset_counters", counters)


@dataclass(frozen=True)
class LandingCorridor:
    """Independently certified, already-eroded reference-center landing volume.

    Physical evidence must cover all admitted horizontal centers, every descent
    height through support contact, entire body/rotors, all admitted attitudes
    and phase clearances. The numeric box itself certifies none of those facts.
    """
    minimum_ned: tuple[float, float, float]
    maximum_ned: tuple[float, float, float]
    support_down_m: float
    support_plane_down_m: float
    body_provenance: str
    physical_provenance: str
    clearance_provenance: str
    frame: str = "local_ned_reference_centers"
    body_point: str = "base_link_origin"

    def __post_init__(self):
        _box(self)
        for name in ("support_down_m", "support_plane_down_m"):
            _number(getattr(self, name), name)
        if not self.minimum_ned[2] < self.support_down_m <= self.maximum_ned[2]:
            raise ValueError("landing corridor must include reference-center support contact")
        if self.support_down_m >= self.support_plane_down_m:
            raise ValueError("support plane must be below the base origin at contact")
        for name in ("body_provenance", "physical_provenance", "clearance_provenance"):
            _identity(getattr(self, name), name)
        if self.frame != "local_ned_reference_centers" or self.body_point != "base_link_origin":
            raise ValueError("unsupported landing frame/body point")


@dataclass(frozen=True)
class PolicyGeometry:
    home: HomeSnapshot
    horizontal_radius_m: float
    height_above_home_m: float
    landing: LandingCorridor
    motion_reserve_m: float
    frame_margin_m: float
    frame_rationale: str

    def __post_init__(self):
        if not isinstance(self.home, HomeSnapshot):
            raise ValueError("frozen geometry requires a Home snapshot")
        if not isinstance(self.landing, LandingCorridor):
            raise ValueError("critical-battery Land requires a known-clear landing corridor")
        for name in ("horizontal_radius_m", "height_above_home_m", "motion_reserve_m", "frame_margin_m"):
            _positive(getattr(self, name), name)
        if max(self.horizontal_radius_m, self.height_above_home_m) > 10000:
            raise ValueError("outer cylinder outside pinned fence parameter range")
        _identity(self.frame_rationale, "local/global frame rationale")


def validate_geometry_policy(expected: dict, context: PolicyGeometry, *,
                             current_home: HomeSnapshot,
                             minimum_ned: tuple[float, float, float],
                             maximum_ned: tuple[float, float, float],
                             required_motion_reserve_m: float,
                             body_provenance: str) -> None:
    """Check full inner geofence plus physical/motion reserve inside outer fence.

    The caller supplies actual valid/current Home/local reference and enforces
    freshness plus sticky change invalidation. A static Home need not publish at
    odometry cadence. PX4's vertical fence is an ABOVE-Home ceiling, not a lower
    floor or a symmetric vertical cylinder; landing support is independent.
    """
    validate_parameter_policy(expected, "geometry compatibility check")
    if not isinstance(context, PolicyGeometry):
        raise ValueError("known-clear battery landing geometry is required")
    if not isinstance(current_home, HomeSnapshot) or current_home != context.home:
        raise ValueError("actual Home/local reference/epoch/reset differs from frozen geometry")
    lower = _vector(minimum_ned, "supervisor minimum")
    upper = _vector(maximum_ned, "supervisor maximum")
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError("supervisor geofence must have positive extent")
    _positive(required_motion_reserve_m, "required motion reserve")
    if required_motion_reserve_m > context.motion_reserve_m:
        raise ValueError("frozen motion reserve does not cover required physical/motion room")
    if (context.horizontal_radius_m != expected["GF_MAX_HOR_DIST"]
            or context.height_above_home_m != expected["GF_MAX_VER_DIST"]):
        raise ValueError("outer cylinder must bind the frozen PX4 fence policy")
    radius = float32_comparison_bounds(expected["GF_MAX_HOR_DIST"])[0]
    height = float32_comparison_bounds(expected["GF_MAX_VER_DIST"])[0]
    reserve = context.motion_reserve_m + context.frame_margin_m
    low = tuple(v - reserve for v in lower)
    high = tuple(v + reserve for v in upper)
    home = context.home.position_ned
    far_x = max(abs(low[0] - home[0]), abs(high[0] - home[0]))
    far_y = max(abs(low[1] - home[1]), abs(high[1] - home[1]))
    # The source uses strict comparisons. Equality at an outer fence is unsafe.
    if hypot(far_x, far_y) >= radius or home[2] - low[2] >= height:
        raise ValueError("whole supervisor geofence plus reserve must fit strictly inside Home cylinder")
    landing = context.landing
    if landing.body_provenance != body_provenance:
        raise ValueError("landing corridor body provenance mismatch")
    if (any(landing.minimum_ned[i] > low[i] or landing.maximum_ned[i] < high[i] for i in (0, 1))
            or landing.minimum_ned[2] > low[2]
            or landing.support_down_m < high[2]):
        raise ValueError("landing corridor must cover entire admitted geofence and reserve through support contact")
