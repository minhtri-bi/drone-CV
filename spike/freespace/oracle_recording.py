"""Schema-four evaluator provenance; raw observations survive collection failures."""

import base64
import hashlib
import json
from types import SimpleNamespace

import numpy as np


def validate_entities(entities: object) -> None:
    if (not isinstance(entities, dict)
            or entities.get("link_pose_convention") != "model_relative"
            or not all(type(entities.get(key)) is int and entities[key] > 0
                       for key in ("model_id", "body_id"))
            or entities["model_id"] == entities["body_id"]
            or not all(isinstance(entities.get(key), str) and entities[key].strip()
                       for key in ("model_name", "body_name"))):
        raise ValueError("INVALID_TRUTH_ENTITY_CONTRACT")


def validate_binding(profile_bytes: bytes, fixture: dict, manifest: dict | None = None) -> dict:
    from drone_cv.oracle import evaluate
    digest = hashlib.sha256(profile_bytes).hexdigest()
    oracle = fixture.get("oracle", {})
    if (oracle.get("profile_sha256") != digest
            or manifest is not None and manifest.get("oracle_profile_sha256") != digest):
        raise ValueError("ORACLE_PROFILE_BINDING_MISMATCH")
    validate_entities(fixture.get("truth_entities"))
    if fixture["truth_entities"]["model_name"] != fixture.get("model_name"):
        raise ValueError("INVALID_TRUTH_ENTITY_CONTRACT")
    if manifest is not None and manifest.get("truth_entities") != fixture["truth_entities"]:
        raise ValueError("TRUTH_ENTITY_BINDING_MISMATCH")
    profile = json.loads(profile_bytes)
    # Validate the frozen geometry and phase, without treating this synthetic pose as evidence.
    evaluate(profile, np.eye(4), oracle["fixture"], oracle["phase"])
    return profile


def packet_evidence(message, wall_ns: int, sim_ns: int) -> dict:
    payload = bytes(message.data)
    return {"payload_base64": base64.b64encode(payload).decode("ascii"),
            "payload_sha256": hashlib.sha256(payload).hexdigest(),
            "wire_layout": {"data_offset": int(message.layout.data_offset), "dim": [
                {"label": dim.label, "size": int(dim.size), "stride": int(dim.stride)}
                for dim in message.layout.dim]},
            "received_wall_ns": wall_ns, "received_sim_ns": sim_ns}


def decode_packet(packet: dict, entities: dict):
    from drone_cv.oracle_truth import body_truth, decode_wire
    try:
        payload = base64.b64decode(packet["payload_base64"], validate=True)
        if hashlib.sha256(payload).hexdigest() != packet["payload_sha256"]:
            raise ValueError("ALIGNMENT_TRUTH_PACKET_HASH_MISMATCH")
        layout = packet["wire_layout"]
        message = SimpleNamespace(data=payload, layout=SimpleNamespace(
            data_offset=layout["data_offset"],
            dim=[SimpleNamespace(**dim) for dim in layout["dim"]]))
        sample = body_truth(decode_wire(message), entities)
        if packet["source_ns"] != sample.source_ns or packet["entities"] != entities:
            raise ValueError("ALIGNMENT_TRUTH_PACKET_IDENTITY_MISMATCH")
        return sample
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"INVALID_ALIGNMENT_TRUTH_PACKET:{error}") from error


class OracleRecording:
    def __init__(self, profile: dict, fixture: dict):
        from drone_cv.oracle import ObserverState
        self.profile = profile
        self.entities = dict(fixture["truth_entities"])
        self.fixture = fixture["oracle"]["fixture"]
        self.phase = fixture["oracle"]["phase"]
        self.state = ObserverState()
        self.first_unsafe = None
        self.first_invalid = None
        self.latest = None

    def fail(self, reason: str) -> None:
        self.state = self.state.fail(reason)

    def receive(self, message, wall_ns: int, sim_ns: int, write_raw, epoch: int = 0):
        from drone_cv.oracle import evaluate
        from drone_cv.oracle_truth import body_truth, decode_wire, stamp_ns
        packet = packet_evidence(message, wall_ns, sim_ns)
        decoded, source_ns, decode_error = None, None, None
        try:
            decoded = decode_wire(message)
            source_ns = stamp_ns(decoded)
        except Exception as error:
            decode_error = str(error)
        packet.update(source_ns=source_ns, entities=dict(self.entities))
        # Even malformed wire content is enqueued before semantic rejection. A storage
        # failure must not skip evaluation of an independently received unsafe pose.
        storage_error = None
        try:
            write_raw(source_ns, decode_error)
        except Exception as error:
            storage_error = error
        try:
            if decode_error is not None:
                raise ValueError(decode_error)
            sample = body_truth(decoded, self.entities)
            packet.update(source_ns=sample.source_ns, entities=dict(self.entities))
            evaluation = evaluate(self.profile, sample.world_from_body, self.fixture, self.phase)
            if evaluation["unsafe"] and self.first_unsafe is None:
                self.first_unsafe = {**packet, "evaluation": evaluation}
            self.state = self.state.observe(evaluation, sim_ns=sample.source_ns, epoch=epoch,
                                            wall_ns=wall_ns, max_gap_ns=self.fixture["max_truth_gap_ns"])
            self.latest = (sample, packet)
        except Exception as error:
            if self.first_invalid is None:
                self.first_invalid = {**packet, "failure": str(error)}
            self.fail(f"TRUTH_CALLBACK:{type(error).__name__}:{error}")
            if storage_error is not None:
                self.fail(f"TRUTH_STORAGE:{storage_error}")
            raise
        if storage_error is not None:
            self.fail(f"TRUTH_STORAGE:{storage_error}")
            raise storage_error
        return sample, packet

    def close_coverage(self, end_sim_ns: int) -> None:
        if self.latest is None:
            self.fail("TRUTH_FINAL_ENDPOINT_MISSING")
            return
        if (self.state.ledger
                and self.state.ledger[0].sim_ns <= end_sim_ns <= self.latest[0].source_ns):
            # A bounded delivered /clock lag can place the endpoint inside already
            # sampled truth coverage. Existing interval gaps remain sticky.
            return
        delta = end_sim_ns-self.latest[0].source_ns
        if delta < 0 or delta > self.fixture["max_truth_gap_ns"]:
            self.fail("TRUTH_FINAL_ENDPOINT_UNCOVERED")

    def finish(self, complete: bool) -> dict:
        result = self.state.finish(complete)
        # Keep the raw/evaluation latch independently of ledger validation or callbacks.
        if self.first_unsafe is not None:
            result.update(safety_outcome="UNSAFE_OBSERVED", observed_unsafe=True)
        return {**result, "first_unsafe": self.first_unsafe, "first_invalid": self.first_invalid}

    def exit_code(self, complete: bool) -> int:
        outcome = self.finish(complete)
        return (1 if outcome["observed_unsafe"] else
                0 if complete and outcome["coverage_complete"] else 2)
