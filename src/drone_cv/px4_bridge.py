"""Pinned PX4 message encoding and explicit, lazy ROS runtime construction.

No imports of rclpy/px4_msgs, publishers, locks or init occur at import. DDS graph
checks are cooperative exclusivity, NOT authentication against a malicious DDS
participant. Deploy only in an operator-controlled isolated ROS domain/network.
"""
from dataclasses import dataclass
from hashlib import sha256
from importlib import import_module
import json
from math import atan2, isfinite, nan
import os
from pathlib import Path
import re
import time

from drone_cv.px4_quaternion import normalize_px4_quaternion
from drone_cv.px4_failsafe import TerminalObservation, read_parameters, validate_policy
from drone_cv.supervisor import (
    Ack, Command, Config, Inputs, Lease, Odometry, OperatorRequest, Output, Permit,
    Stage, State, Status, revoke, transition,
)
from drone_cv.timing import Clock, Stamp

FIRMWARE_PIN = "61907962c9220437ec01a98fa26bfbf5cbb254b5"
MESSAGES_PIN = "8fb8277cf95c4ba87094146bc8922e76c4ac0799"
IMAGE_PIN = "sha256:061a39905d2d84d40859bb46b67048241b6dacf2ed3983889da7f5fd3e718285"
FIRMWARE_ROOT = f"https://raw.githubusercontent.com/PX4/PX4-Autopilot/{FIRMWARE_PIN}"
MESSAGES_ROOT = f"https://raw.githubusercontent.com/PX4/px4_msgs/{MESSAGES_PIN}/msg"
PINNED_SOURCES = (
    *(f"{MESSAGES_ROOT}/{name}.msg" for name in (
        "OffboardControlMode", "TrajectorySetpoint", "VehicleCommand",
        "VehicleCommandAck", "VehicleStatus", "VehicleOdometry")),
    f"{FIRMWARE_ROOT}/src/modules/commander/Commander.cpp",
    f"{FIRMWARE_ROOT}/src/modules/commander/px4_custom_mode.h",
    f"{FIRMWARE_ROOT}/src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp",
    f"{FIRMWARE_ROOT}/src/modules/uxrce_dds_client/dds_topics.yaml",
    f"{FIRMWARE_ROOT}/src/modules/uxrce_dds_client/dds_topics.h.em",
    f"{FIRMWARE_ROOT}/src/modules/uxrce_dds_client/utilities.hpp",
)
CONTROL_TOPICS = (
    "offboard_control_mode", "trajectory_setpoint", "vehicle_command",
    "vehicle_command_mode_executor", "goto_setpoint", "vehicle_attitude_setpoint",
    "vehicle_rates_setpoint", "vehicle_thrust_setpoint", "vehicle_torque_setpoint",
    "actuator_motors", "actuator_servos", "manual_control_input",
    "register_ext_component_request", "config_overrides_request", "setpoint_config",
)
OUTPUT_TYPES = {"offboard_control_mode": "OffboardControlMode",
                "trajectory_setpoint": "TrajectorySetpoint", "vehicle_command": "VehicleCommand"}
INPUT_TYPES = {"vehicle_odometry": "VehicleOdometry", "vehicle_status_v4": "VehicleStatus",
               "vehicle_command_ack_v1": "VehicleCommandAck"}


def message_types():
    """Load generated classes only; never rclpy.init or a publisher."""
    types = import_module("px4_msgs.msg")
    expected = {"VehicleStatus": ("MESSAGE_VERSION", 4),
                "VehicleCommandAck": ("MESSAGE_VERSION", 1),
                "TrajectorySetpoint": ("MESSAGE_VERSION", 0),
                "VehicleCommand": ("VEHICLE_CMD_DO_SET_MODE", 176),
                "VehicleOdometry": ("POSE_FRAME_NED", 1)}
    for name, (constant, value) in expected.items():
        if getattr(getattr(types, name), constant) != value:
            raise ValueError(f"generated PX4 contract mismatch: {name}.{constant}")
    return types


def encode(output: Output, timestamp_us: int, cfg: Config, types=None) -> dict:
    """Encode generated messages offline; priming is heartbeat-only outside Offboard."""
    if type(timestamp_us) is not int or not 0 < timestamp_us < 2**64:
        raise ValueError("publish timestamp must be positive uint64 simulation microseconds")
    if output.position_ned is not None and output.velocity_ned is not None:
        raise ValueError("setpoint cannot control position and velocity together")
    if not output.heartbeat and (output.position_ned is not None or output.velocity_ned is not None):
        raise ValueError("a supervised setpoint requires its heartbeat")
    types = message_types() if types is None else types
    result = {}
    if output.heartbeat:
        values = output.position_ned if output.position_ned is not None else output.velocity_ned
        if values is not None and (len(values) != 3 or not all(isfinite(v) for v in values)
                or output.yaw_ned is None or not isfinite(output.yaw_ned)):
            raise ValueError("physical setpoint must be finite")
        mode = types.OffboardControlMode()
        mode.timestamp = timestamp_us
        for field in ("position", "velocity", "acceleration", "attitude", "body_rate",
                      "thrust_and_torque", "direct_actuator"):
            setattr(mode, field, False)
        mode.position = output.velocity_ned is None
        mode.velocity = output.velocity_ned is not None
        result["offboard_control_mode"] = mode
        if values is not None:
            point = types.TrajectorySetpoint()
            point.timestamp = timestamp_us
            for field in ("position", "velocity", "acceleration", "jerk"):
                setattr(point, field, [nan, nan, nan])
            point.yaw, point.yawspeed = float(output.yaw_ned), nan
            field = "position" if mode.position else "velocity"
            setattr(point, field, [float(v) for v in values])
            result["trajectory_setpoint"] = point
    if output.vehicle_command is not None:
        if output.vehicle_command not in ("offboard", "arm", "loiter"):
            raise ValueError("unsupported vehicle command")
        command = types.VehicleCommand()
        command.timestamp = timestamp_us
        # Pinned Commander casts DO_SET_MODE param1/2/3; arm reads param1/2.
        # Use explicit finite zeros for consumed unused mode/force parameters.
        for i in range(1, 8):
            setattr(command, f"param{i}", nan)
        command.command = 400 if output.vehicle_command == "arm" else 176
        command.param1 = 1.
        command.param2 = 0. if output.vehicle_command == "arm" else (6. if output.vehicle_command == "offboard" else 4.)
        command.param3 = 3. if output.vehicle_command == "loiter" else 0.
        command.target_system, command.target_component = cfg.target_system, cfg.target_component
        command.source_system, command.source_component = cfg.source_system, cfg.source_component
        command.confirmation, command.from_external = 0, True
        result["vehicle_command"] = command
    return result


def decode_odometry(msg, epoch: str, wall_s: float) -> Odometry:
    if msg.pose_frame != 1 or msg.velocity_frame != 1:
        raise ValueError("odometry must be NED pose and NED velocity")
    normalized = normalize_px4_quaternion(msg.q)
    w, x, y, z = normalized.wxyz
    yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return Odometry(Stamp(epoch, int(msg.timestamp_sample), msg.timestamp_sample/1e6, wall_s),
                    tuple(msg.position), tuple(msg.velocity), yaw, int(msg.reset_counter),
                    tuple(msg.position_variance), tuple(msg.velocity_variance),
                    normalized.squared_norm_error)


def decode_status(msg, epoch: str, wall_s: float) -> Status:
    return Status(Stamp(epoch, int(msg.timestamp), msg.timestamp/1e6, wall_s),
                  int(msg.nav_state), int(msg.arming_state),
                  bool(msg.failsafe or msg.failsafe_and_user_took_over or msg.failsafe_defer_state == 2),
                  int(msg.executor_in_charge), bool(msg.pre_flight_checks_pass),
                  int(msg.system_id), int(msg.component_id), msg.nav_state_timestamp/1e6,
                  msg.armed_time/1e6)


def decode_ack(msg, epoch: str, wall_s: float) -> Ack:
    # Pinned Commander::answer_command zero-initializes from_external (false).
    return Ack(Stamp(epoch, int(msg.timestamp), msg.timestamp/1e6, wall_s),
               int(msg.command), int(msg.result), int(msg.target_system),
               int(msg.target_component), bool(msg.from_external))


def decode_json(payload: str, kind: str, epoch: str, wall_s: float):
    """Strict bounded local-domain JSON interface, preserving acquisition expiry.

    Wire stamps are {epoch, sequence, sim_s}; receive times are local monotonic.
    Permits require the ORIGINAL observation capture_stamp, not a verdict stamp.
    Separate command/permit/lease/request topics are required; unknown keys fail.
    """
    if len(payload.encode("utf-8")) > 8192:
        raise ValueError("input JSON exceeds 8192 bytes")
    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ValueError("duplicate JSON field")
            value[key] = item
        return value
    data = json.loads(payload, object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    if not isinstance(data, dict):
        raise ValueError("input must be an object")
    def stamp_value(raw):
        if not isinstance(raw, dict) or set(raw) != {"epoch", "sequence", "sim_s"}:
            raise ValueError("wire stamp requires epoch, sequence and sim_s only")
        if raw["epoch"] != epoch:
            raise ValueError("wire clock epoch mismatch")
        return Stamp(raw["epoch"], raw["sequence"], raw["sim_s"], wall_s)
    stamp = stamp_value(data.pop("stamp"))
    if kind == "permit":
        data["capture_stamp"] = stamp_value(data.pop("capture_stamp"))
    classes = {"command": Command, "permit": Permit, "lease": Lease, "request": OperatorRequest}
    return classes[kind](stamp=stamp, **data)


@dataclass(frozen=True)
class RuntimeAuthorization:
    owner: str
    namespace: str
    epoch: str
    firmware: str
    messages: str
    image: str
    confirmation: str
    lock_directory: str
    discovery_wall_s: float
    discovery_stable_s: float
    tick_wall_s: float
    target: str
    parameter_policy: dict
    policy_rationale: str
    readback_wall_s: float
    terminal_observation_wall_s: float
    instance: int

    def validate(self, cfg: Config):
        if self.target != "exploratory_simulation" or cfg.envelope.target != self.target:
            raise ValueError("runtime requires explicit exploratory simulation bounds; no offline fixture or hardware")
        if type(self.instance) is not int or self.instance != 0:
            raise ValueError("pinned local readback supports explicit PX4 instance zero only")
        validate_policy(self.parameter_policy, self.policy_rationale, self.readback_wall_s)
        if (not isfinite(self.terminal_observation_wall_s)
                or self.terminal_observation_wall_s <= 0):
            raise ValueError("bounded terminal observation deadline required")
        if self.owner != cfg.owner or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", self.owner):
            raise ValueError("startup owner must match a valid unique ROS node name")
        if not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", self.namespace):
            raise ValueError("an explicit absolute vehicle namespace is required")
        if not self.epoch or len(self.epoch) > 128:
            raise ValueError("explicit clock epoch is required")
        if (self.firmware, self.messages, self.image) != (FIRMWARE_PIN, MESSAGES_PIN, IMAGE_PIN):
            raise ValueError("startup manifest must attest the exact pinned environment")
        if self.confirmation != f"ALLOW_CONTROL_PUBLISHERS:{self.namespace}:{self.owner}":
            raise ValueError("explicit operator publisher-construction authorization required")
        for value in (self.discovery_wall_s, self.discovery_stable_s, self.tick_wall_s):
            if not isfinite(value) or value <= 0:
                raise ValueError("runtime deadlines must be finite and positive")
        if (self.discovery_stable_s >= self.discovery_wall_s
                or self.tick_wall_s >= cfg.max_tick_gap_wall_s):
            raise ValueError("invalid discovery/tick deadline ordering")
        if not Path(self.lock_directory).is_dir():
            raise ValueError("operator-provided lock directory must already exist")


class ProcessLock:
    """OS-held lock survives no process death; never unlink the shared inode."""
    def __init__(self, directory, namespace):
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        key = sha256(f"{domain}:{namespace}".encode()).hexdigest()
        self.file = open(Path(directory)/f"drone-cv-offboard-{key}.lock", "a+b")
        try:
            if os.name == "nt":
                import msvcrt
                self.file.seek(0)
                if not self.file.read(1):
                    self.file.write(b"0")
                    self.file.flush()
                self.file.seek(0)
                msvcrt.locking(self.file.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.file.close()
            raise

    def close(self):
        self.file.close()


def graph_exclusive(node, namespace, owner, planner, safety, own_gids=None) -> bool:
    """Missing endpoints, duplicate identities, aliases or API errors fail closed."""
    try:
        own_gids = {} if own_gids is None else own_gids
        nodes = node.get_node_names_and_namespaces()
        for name in (owner, planner, safety, owner + "_operator"):
            if nodes.count((name, namespace)) != 1:
                return False
        names = dict(node.get_topic_names_and_types())
        for topic in names:
            if not topic.startswith(namespace + "/fmu/in/"):
                continue
            leaf = topic.rsplit("/", 1)[1]
            if re.sub(r"_v\d+$", "", leaf) not in CONTROL_TOPICS:
                continue
            pubs = node.get_publishers_info_by_topic(topic)
            key = leaf if leaf in own_gids else None
            if key is None and pubs:
                return False
            if key is not None:
                if len(pubs) != 1 or bytes(pubs[0].endpoint_gid) != own_gids[key]:
                    return False
                if (pubs[0].node_name, pubs[0].node_namespace) != (owner, namespace):
                    return False
        for key, type_name in INPUT_TYPES.items():
            pubs = node.get_publishers_info_by_topic(f"{namespace}/fmu/out/{key}")
            if len(pubs) != 1 or pubs[0].topic_type != f"px4_msgs/msg/{type_name}":
                return False
        for key, type_name in OUTPUT_TYPES.items():
            topic = f"{namespace}/fmu/in/{key}"
            readers = node.get_subscriptions_info_by_topic(topic)
            if len(readers) != 1 or readers[0].topic_type != f"px4_msgs/msg/{type_name}":
                return False
            if key in own_gids and topic not in names:
                return False
        for leaf, source in (("command", planner), ("permit", safety),
                             ("lease", owner + "_operator"), ("request", owner + "_operator")):
            pubs = node.get_publishers_info_by_topic(f"{namespace}/supervisor/{leaf}")
            if (len(pubs) != 1 or pubs[0].topic_type != "std_msgs/msg/String"
                    or (pubs[0].node_name, pubs[0].node_namespace) != (source, namespace)):
                return False
        return len(node.get_publishers_info_by_topic("/clock")) == 1
    except Exception:
        return False


class RosBridge:
    """Opt-in single executor adapter. Construction may create control publishers.

    Caller must own rclpy.init/shutdown and a SingleThreadedExecutor. First run
    explicit discovery with subscriptions only; publisher construction requires
    sustained exclusivity. Each tick rechecks it before encoding any output.
    """
    def __init__(self, cfg: Config, authorization: RuntimeAuthorization):
        authorization.validate(cfg)  # BEFORE ROS import, init, node or publisher.
        self.cfg, self.authorization = cfg, authorization
        self.state, self.latest, self.publishers = State(), {}, {}
        self.lock = ProcessLock(authorization.lock_directory, authorization.namespace)
        self.started_wall = time.monotonic()
        self.node, self.timer = None, None
        self.terminal_observation = None
        self.parameter_readback = None
        self.status_trace, self.ack_trace, self.publication_trace = (), (), ()
        try:
            rclpy = import_module("rclpy")
            if not rclpy.ok():
                raise RuntimeError("caller must explicitly initialize rclpy")
            from rclpy.parameter import Parameter
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from rclpy.clock import Clock as RosClock, ClockType
            from rosgraph_msgs.msg import Clock as ClockMessage
            from std_msgs.msg import String
            self.types = message_types()
            self.node = rclpy.create_node(authorization.owner, namespace=authorization.namespace,
                parameter_overrides=[Parameter("use_sim_time", value=True)])
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.clock_value, self.clock_progress = None, time.monotonic()
            self.node.create_subscription(ClockMessage, "/clock", self._clock, qos)
            for key, name in INPUT_TYPES.items():
                kind = {"vehicle_odometry": "odometry", "vehicle_status_v4": "status",
                        "vehicle_command_ack_v1": "ack"}[key]
                self.node.create_subscription(getattr(self.types, name),
                    f"{authorization.namespace}/fmu/out/{key}",
                    lambda msg, kind=kind: self._receive_px4(kind, msg), qos)
            for kind in ("command", "permit", "lease", "request"):
                self.node.create_subscription(String, f"{authorization.namespace}/supervisor/{kind}",
                    lambda msg, kind=kind: self._receive_json(kind, msg.data), 1)
            deadline, stable = time.monotonic()+authorization.discovery_wall_s, None
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=min(.05, authorization.tick_wall_s))
                valid = self._graph() and self.clock_value is not None
                stable = (stable if stable is not None else time.monotonic()) if valid else None
                if stable is not None and time.monotonic()-stable >= authorization.discovery_stable_s:
                    break
            else:
                raise RuntimeError("exclusive DDS discovery deadline expired; no control publishers created")
            if self.state.stage == Stage.REVOKED:
                raise RuntimeError(self.state.reason)
            self.parameter_readback = read_parameters(
                authorization.parameter_policy, authorization.policy_rationale,
                authorization.readback_wall_s, authorization.firmware)
            # Readback may take time: recheck graph before creating publishers.
            if not self._graph():
                raise RuntimeError("DDS ownership changed during parameter readback")
            for key, name in OUTPUT_TYPES.items():
                self.publishers[key] = self.node.create_publisher(getattr(self.types, name),
                    f"{authorization.namespace}/fmu/in/{key}", qos)
            deadline = time.monotonic() + authorization.discovery_wall_s
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=min(.05, authorization.tick_wall_s))
                gids = {}
                for key in OUTPUT_TYPES:
                    endpoints = self.node.get_publishers_info_by_topic(
                        f"{authorization.namespace}/fmu/in/{key}")
                    if (len(endpoints) == 1 and endpoints[0].node_name == authorization.owner
                            and endpoints[0].node_namespace == authorization.namespace):
                        gids[key] = bytes(endpoints[0].endpoint_gid)
                if len(gids) == len(OUTPUT_TYPES):
                    self.gids = gids
                    if self._graph():
                        break
            else:
                raise RuntimeError("post-construction graph verification failed; nothing published")
            self.timer = self.node.create_timer(authorization.tick_wall_s, self._tick,
                                               clock=RosClock(clock_type=ClockType.STEADY_TIME))
        except BaseException:
            self.close()
            raise

    def _graph(self):
        auth = self.authorization
        return graph_exclusive(self.node, auth.namespace, auth.owner,
                               self.cfg.planner, self.cfg.safety, getattr(self, "gids", None))

    def _clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec/1e9
        if self.clock_value is not None and value < self.clock_value:
            self.state, _ = revoke(self.state, "CLOCK_RESET")
        if self.clock_value is None or value > self.clock_value:
            self.clock_progress = time.monotonic()
        self.clock_value = value

    def _receive_px4(self, kind, msg):
        try:
            decoder = {"odometry": decode_odometry, "status": decode_status, "ack": decode_ack}[kind]
            value = decoder(msg, self.authorization.epoch, time.monotonic())
            if kind in ("status", "ack"):
                attribute = kind + "_trace"
                setattr(self, attribute, (*getattr(self, attribute, ())[-255:], value))
            terminal = getattr(self, "terminal_observation", None)
            if kind == "status" and terminal is not None:
                self.terminal_observation = terminal.observe(value)
            previous = self.latest.get(kind)
            if previous is not None:
                if (value.stamp.sequence <= previous.stamp.sequence
                        or value.stamp.sim_s <= previous.stamp.sim_s):
                    raise ValueError("nonadvancing source timestamp")
                if kind == "odometry" and value.reset_counter != previous.reset_counter:
                    raise ValueError("estimator reset")
            if kind == "status":
                # Never let a later healthy packet erase observed loss of authority.
                if (value.system_id, value.component_id) != (self.cfg.target_system, self.cfg.target_component):
                    raise ValueError("unexpected PX4 identity")
                if self.state.stage == Stage.PRIMING and value.nav_state == 14:
                    raise ValueError("unrequested PX4 Offboard entry")
                if value.failsafe or value.executor_in_charge != 0:
                    raise ValueError("PX4 failsafe or competing mode executor")
                if (self.state.stage in (Stage.OFFBOARD_DISARMED, Stage.ARM_WAIT, Stage.ACTIVE)
                        and value.nav_state != 14):
                    raise ValueError("PX4 left Offboard")
                if (self.state.stage == Stage.MODE_WAIT and previous is not None
                        and previous.nav_state == 14 and value.nav_state != 14):
                    raise ValueError("PX4 left Offboard before mode ACK")
                if (self.state.stage == Stage.ARM_WAIT and previous is not None
                        and previous.arming_state == 2 and value.arming_state != 2):
                    raise ValueError("PX4 disarmed before arm ACK")
                if self.state.stage == Stage.ACTIVE and value.arming_state != 2:
                    raise ValueError("PX4 disarmed")
                if (self.state.stage in (Stage.PRIMING, Stage.MODE_WAIT, Stage.OFFBOARD_DISARMED)
                        and value.arming_state != 1):
                    raise ValueError("unrequested PX4 arm")
            if kind == "ack" and self.state.pending_since is not None:
                pending = self.state.pending_since
                command = 400 if self.state.stage == Stage.ARM_WAIT else 176
                if (self.state.stage in (Stage.MODE_WAIT, Stage.ARM_WAIT, Stage.HANDOFF_WAIT)
                        and value.command == command and value.result != 0
                        and (value.target_system, value.target_component)
                            == (self.cfg.source_system, self.cfg.source_component)
                        and not value.from_external and value.stamp.sim_s > pending.sim_s
                        and value.stamp.received_wall_s > pending.wall_s):
                    raise ValueError(f"PX4 command rejected: {value.result}")
            self.latest[kind] = value
        except (ValueError, TypeError, OverflowError) as error:
            self.state, _ = revoke(self.state, f"INVALID_{kind.upper()}:{error}")

    def _receive_json(self, kind, payload):
        try:
            value = decode_json(payload, kind, self.authorization.epoch, time.monotonic())
            previous = self.latest.get(kind)
            if previous is not None and (value.stamp.sequence <= previous.stamp.sequence
                                         or value.stamp.sim_s <= previous.stamp.sim_s):
                raise ValueError("nonadvancing source sequence/timestamp")
            self.latest[kind] = value
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.state, _ = revoke(self.state, f"INVALID_{kind.upper()}:{error}")

    def _tick(self):
        if self.state.stage in (Stage.REVOKED, Stage.RELEASED):
            self._stop_publishers()
            return
        try:
            if self.node is None:
                raise RuntimeError("adapter node is closed")
            wall = time.monotonic()
            if wall - self.started_wall >= self.cfg.session_timeout_s:
                self.state, _ = revoke(self.state, "RUNTIME_SESSION_TIMEOUT")
                self._stop_publishers()
                return
            if wall-self.clock_progress > self.cfg.clock_stall_wall_s:
                self.state, _ = revoke(self.state, "CLOCK_SOURCE_STALLED")
                self._stop_publishers()
                return
            now = Clock(self.authorization.epoch, self.node.get_clock().now().nanoseconds/1e9, wall)
            data = Inputs(now, self.latest.get("odometry"), self.latest.get("status"),
                self.latest.get("command"), self.latest.get("permit"), self.latest.get("lease"),
                self._graph(), self.latest.get("request"), self.latest.get("ack"))
            self.state, output = transition(self.state, data, self.cfg)
            messages = encode(output, int(now.sim_s*1e6), self.cfg, self.types)
            # No callback interleaves the graph check and this send in the single executor.
            for key in ("trajectory_setpoint", "offboard_control_mode", "vehicle_command"):
                if key in messages:
                    self.publishers[key].publish(messages[key])
                    self.publication_trace = (*getattr(self, "publication_trace", ())[-255:],
                        (now.sim_s, wall, key, output.vehicle_command))
            if self.state.stage in (Stage.REVOKED, Stage.RELEASED):
                self._stop_publishers()
        except Exception as error:
            self.state, _ = revoke(self.state, f"ADAPTER_ERROR:{error}")
            self._stop_publishers()

    def _stop_publishers(self):
        if (self.state.stage in (Stage.REVOKED, Stage.RELEASED)
                and getattr(self, "terminal_observation", None) is None):
            wall = time.monotonic()
            self.terminal_observation = TerminalObservation(
                wall, wall + self.authorization.terminal_observation_wall_s,
                self.authorization.epoch, self.cfg.target_system,
                self.cfg.target_component, self.state.reason)
        if self.node is None:
            return
        for pub in self.publishers.values():
            self.node.destroy_publisher(pub)
        self.publishers.clear()
        if self.timer is not None:
            self.timer.cancel()

    def terminal_finished(self) -> bool:
        if self.state.stage not in (Stage.REVOKED, Stage.RELEASED):
            return False
        self._stop_publishers()
        terminal = self.terminal_observation
        if terminal is None:
            raise RuntimeError("terminal observation was not initialized")
        return time.monotonic() >= terminal.deadline_wall_s

    def close(self):
        if self.node is not None:
            self._stop_publishers()
            self.node.destroy_node()
            self.node = None
        self.lock.close()
