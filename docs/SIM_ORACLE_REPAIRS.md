# Offline moving-oracle repair contract

This implements the code/offline portion of R3 and the N5/N6 reporting and
supplemental-checker cutovers in [SIM_REPAIR_PLAN.md](SIM_REPAIR_PLAN.md).
It does not launch ROS, Gazebo or PX4, publish control, discover a live graph,
create a recording, establish DDS access control, or authorize flight. Existing
static fixtures, captured bags and published reports are unchanged.

## Actual entry point and compatibility

`python scripts/g1_oracle.py evaluate --profile PROFILE.json --fixture FIXTURE.json --observations OBSERVATIONS.json --output NEW_DIRECTORY`

As before, configure `PYTHONPATH=src`; output creation is exclusive. A fixture
without `moving_contract` uses the original static observation array and original
sampled endpoint ledger. `replay` retains its historical behavior. A fixture with
`moving_contract` selects `drone_cv.oracle_observation.MovingObserverState` in the
actual `evaluate` path. Its observations document must contain exactly two arrays:
`observations` and `discovery`. Replay rejects a moving contract because existing
bags do not contain the required independently bound phase/discovery evidence;
there is no implicit upgrade or synthesized dynamic certificate.

The report retains the sampled geometry ledger and adds `moving_intervals`,
`phase_ledger`, `discovery`, and `motion_assumptions`. All input files retain CLI
SHA-256 bindings. `sampling_scope` identifies the declared moving contract. Exit
status remains 1 for retained demonstrated sampled unsafe geometry, 2 for
unknown/incomplete/error coverage, and 0 for complete conditional offline
coverage without a violation. `flight_admission` and
`live_acceptance_certified` remain false even on exit 0.

Moving intervals and their summary emit
`clearance_conditional_on_declared_motion_bounds`, not
`continuous_clearance_certified`. The inherited static-only flag is also removed
from moving reports' sampled `coverage_intervals`; those remain endpoint coverage,
not moving enclosure results. There is no legacy alias. The static evaluate/replay
path retains its existing false certificate flag and sampled-only semantics.
Historical reports and raw evidence are not migrated.

Each moving interval and summary includes the supplied
`motion_bounds_provenance` (null when absent), `motion_bounds_calibrated: false`,
`independent_motion_bound_evidence_present: false`, and
`motion_bounds_scope: supplied_offline_intervals_only_no_independent_bound_evidence`.
These fields describe the current implementation's lack of independent bound
evidence, not a judgment that a caller's provenance text proves calibration.
A caller-provided `calibrated` boolean or source label cannot change them or
grant flight authority. Raw `motion_assumptions` are declarations only.
Malformed moving initialization still emits a non-admitting conditional report,
not a silent fallback to the static schema.

## Frozen schema

Keep the original profile schema, complete component AABBs, surfaces, support
zones and phase clearances. Moving fixtures require `rest`, `takeoff`,
`navigation` and `landing` clearances, `max_truth_gap_ns`, and this additional
contract (numbers below are **synthetic example constraints**, not calibrated
values or permission to use them operationally):

```json
{
  "schema": 1,
  "truth_source": "truth/fixture-1",
  "telemetry_source": "px4/fixture-1",
  "support_source": "support/fixture-1",
  "max_source_age_ns": 0,
  "max_wall_gap_ns": 1000000000,
  "max_observations": 100,
  "motion": {
    "translation_speed_m_s": 4.0,
    "angular_speed_rad_s": 3.141592653589793,
    "provenance": "Analytic fixture constraints; not stopping calibration"
  },
  "schedule": [
    {"phase": "rest", "start_sim_ns": 0, "end_sim_ns": 2000000000},
    {"phase": "takeoff", "start_sim_ns": 2000000000, "end_sim_ns": 3000000000},
    {"phase": "navigation", "start_sim_ns": 3000000000, "end_sim_ns": 4000000000},
    {"phase": "landing", "start_sim_ns": 4000000000, "end_sim_ns": 5000000000},
    {"phase": "rest", "start_sim_ns": 5000000000, "end_sim_ns": 6000000000}
  ],
  "discovery": {
    "source": "graph/fixture-1",
    "max_gap_ns": 1000000000,
    "max_snapshots": 100,
    "max_endpoints": 10,
    "allowed_readers": [
      {"gid": "endpoint-1", "node": "oracle", "namespace": "/evaluator", "topic": "/truth"}
    ]
  }
}
```

The three phase/truth source identities must be distinct. Schedules are frozen,
contiguous half-open simulation-time windows in exactly
`rest -> takeoff -> navigation -> landing -> rest` order. No phase text from the
planner or supervisor is consumed. The declared collection may cover a subset of
these windows; a successful subset is not full-mission acceptance. Navigation and
takeoff margins cannot be below 0.5 m. Support allowlist phases are restricted to
rest/landing; the full conservative body/rotor/gear profile is retained.

Each observation contains nonnegative integer `sim_ns`, `wall_ns`, `epoch`,
`truth_source`, `source_ns`, a rigid `world_from_body`, plus independent:

- `telemetry`: `source`, `source_ns`, `epoch`, boolean `armed`, boolean `landed`,
  and `nav_state`;
- `support`: `source`, `source_ns`, `epoch`, and a `contacts` array of
  `[part_id, surface_id]` pairs (possibly empty).

Truth acquisition `source_ns` must equal `sim_ns`. Telemetry/support must belong
to the same epoch, cannot lead truth, and cannot exceed `max_source_age_ns`.
Expected navigation state labels are respectively `DISARMED`, `TAKEOFF`,
`OFFBOARD`, `LAND`, `DISARMED`; these are the offline contract's normalized
semantic labels, **not** claims about pinned PX4 wire enum values. Both rest
windows require disarmed/landed; navigation requires armed/not-landed; takeoff
and landing require armed. An independent support report must agree with the
actual frozen gear/surface intersection and full contact zone. Rest with a
support allowlist requires observed support. Landing without corroborated landed
support retains navigation clearance and no contact exemption.

A discovery snapshot contains `wall_ns`, `source`, `readers` (full endpoint
identities with exactly `gid`, `node`, `namespace`, `topic`), and optional `error`
(null for success). Empty reader sets are permitted. Identity comparison uses all
four fields, not just node names. Snapshot order and every adjacent wall-time gap
are checked; first/last graph snapshots must cover the supplied truth endpoints.
Collection completion is still an explicit caller assertion; the CLI supplies it
only after consuming the two offline arrays.

## Interval enclosure and outcomes

For each complete component, radius `r` is the maximum Euclidean norm of **all
eight body-frame AABB corners**, including component offsets from the body
origin. With interval duration `dt`, the first-pose world AABB is expanded in
all axes by:

`v_max*dt + 2*r*sin(min(omega_max*dt, pi)/2)`.

The translation speed bounds the path throughout the interval, including curved
excursions that return to an identical endpoint. Angular speed is a declared
bound on the entire rotational path, not inferred from endpoint angle difference.
The endpoint translation and shortest SO(3) angle must also obey these bounds.
No acceleration term is added because this contract bounds speed throughout,
not just initial velocity. An acceleration-only assumption is not accepted as a
replacement. Exact declared rate and gap boundaries are inclusive; there is no
new tolerance that enlarges a rate or timing limit.

`CLEAR` means every expanded component/surface pair meets the authorized phase
margin, or a gear-only support enclosure is wholly contained in one frozen
allowlisted contact zone with corroborated landed/support observations at both
endpoints. Only that conditional success sets the interval's
`clearance_conditional_on_declared_motion_bounds` true. Body/rotor intersections
are never exempted. An interval crossing a phase window is UNKNOWN rather than
extrapolating its exemption. Landing before landed/support confirmation retains
the navigation margin. A full expanded support intersection that escapes its zone
does not establish conditional clearance, even when both sampled contacts fit.

`POTENTIAL_VIOLATION` means the conservative enclosure cannot establish
clearance. It is **not** demonstrated intersection along the actual trajectory
and never a Gazebo physical-contact event. Thin crossing, rotation-only corner
sweeps, and intermediate curved excursions can therefore be detected as
potential violations with both sampled endpoints clear.

`SAMPLED_VIOLATION` records a demonstrated violation of the declared sampled
conservative geometry, not physical contact. Its unsafe latch survives later
source loss, discovery failure or incomplete execution. Missing/invalid phase
evidence never grants an exemption, but a collision that could be a valid gear
support under an unknown phase is UNKNOWN, not a newly demonstrated violation.
Body/rotor intersections and contacts outside every possible support zone remain
known unsafe. Invalid truth identity/timestamp cannot create a new demonstrated
violation; previously known unsafe evidence is retained. Consult
`phase_ledger.known_sampled_violation` to distinguish that conclusion from the
raw conservative `evaluation.unsafe` flag under uncertain phase evidence.

Missing motion bounds, source/epoch loss, nonadvancing stamps, source/wall gaps,
rate-bound contradictions, invalid phase order and interrupted observations
produce UNKNOWN/incomplete coverage. Missing motion bounds are retained as an
explicit UNKNOWN interval; malformed supplied bounds/provenance reject contract
initialization. No later recovery erases a prior gap or unsafe latch. Lost phase
authority remains invalid for the rest of that immutable observer; changing the
epoch, skipping/backtracking a phase or losing required phase observations never
re-enables later contact exemptions. The summary's conditional flag requires
nonempty, fully CLEAR interval coverage, complete sampled/discovery coverage and
no retained observation failures. An individually CLEAR interval can remain
conditional true while incomplete collection or a discovery failure makes the
summary false. Neither result certifies bounds, unobserved time or a live flight.

## Bounded cooperative discovery, not security

The ledger retains every admitted graph observation and transient unexpected
reader, even if that reader disappears before the final snapshot. Graph errors,
malformed identities, gaps, missing endpoints and capacity overflow are sticky.
`cooperative_coverage_complete` means observations meet the declared cadence and
endpoint coverage; it does **not** prove that no endpoint existed between graph
observations. `dds_security_certified` is always false. Domain IDs and node names
are not credentials. The evaluator neither grants DDS permissions nor asserts
live domain isolation.

Limits are frozen: at most 10,000 truth observations, 10,000 graph snapshots,
256 endpoints per snapshot, and 256 parts/surfaces each. Individual observation
and discovery payloads are limited to 65,536 UTF-8 bytes. Error retention is
limited to 64 unique reasons of 512 characters each. Overflow rejects further
coverage; a newly observed geometric violation at the truth-ledger count limit
still retains the unsafe latch. Immutable JSON snapshots detach all nested input
objects; state transitions return new frozen states.

## Supplemental roadmap gates

`scripts/check_project.py` declares the four approved supplemental policies in
one immutable `SUPPLEMENTAL_GATES` tuple. Each entry contains its prerequisite ID,
exact/prefix target selector and guarded statuses. Both enforcement and the
exported portable roadmap table consume that same tuple; the table is not
independently maintained prose and does not modify native dependencies.

Flight-oracle preparation guards `g1-flight` at both `in_progress` and `done`.
The two evidence preparations guard completion of every `g2-` task, including
future IDs. The depth-path preparation guards calibration completion, not
acquisition start. Missing or incomplete preparations fail closed at their
declared statuses. The evidence/y branch does not block unrelated G1 work.
Geometry contradictions separately block affected-profile use, not an invented
generic dependency. No gate is completed by changing these declarations.

## Offline verification handoff

No tests, lint, builds, formatter or simulator were executed by the delegated
implementation worker. The integrator should run once after all sibling edits:

```text
python -m pytest -q tests/test_oracle.py tests/test_oracle_observation.py tests/test_g1_oracle_cli.py tests/test_project_integrity.py
python -m pytest -q tests/test_oracle_observation.py -k cli
python scripts/check_project.py --export-roadmap
python scripts/check_project.py
```

The CLI regressions exercise report/exit-code behavior with synthetic thin
crossing, rotation, curved excursion, missing bounds, source gap/loss, retained
unsafe/source loss, skipped phase, disappearing unexpected reader, and clear
positive cases. Tests cover conditional-only outputs, ignored calibration
assertions, malformed moving initialization, exact gap/rate boundaries, epoch
loss, support containment, immutable inputs and bounded capacity. Checker tests
exercise all policy/status boundaries and change each policy's prerequisite,
selector and guarded statuses to establish shared enforcement/render behavior.
These are callable verification commands, not claims of passing execution.
The exporter must run before the ordinary integrity check because the generated
table changes the portable view; it does not mutate the native roadmap.

## Runtime prerequisites still missing

Before any planner participates in acceptance, separately design and verify an
evaluator-only process/domain truth transport, exposing only the allowed
localization/health interface to control consumers. Bind actual endpoint GIDs,
node/namespace/topic identities, graph source identity and a justified discovery
cadence/failure recorder. Actual continuous bounded graph observations are not
implemented by this offline module, and historical start/end snapshots cannot
supply them.

Bind independently acquired pinned PX4 arming/landed/navigation telemetry and
support/contact observations, define and verify wire-enum-to-semantic-label
conversion, and freeze their source identities, acquisition stamps, epoch,
allowed ages and mission schedule before collection. The historical static bags
do not supply this complete contract.

Freeze justified full-path translation/angular speed limits and provenance for
the intended trials. Synthetic values are not stopping calibration. New unarmed
recordings/fault trials, live graph/isolation checks, moving truth trials and
armed flight/fallback testing require separate execution authorization and their
own retained raw evidence. No live isolation, unarmed fault acceptance, flight,
stopping calibration or G1 completion is claimed by these changes.
