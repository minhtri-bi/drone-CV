# drone-CV — project instructions

## Goal and current work

- Self-study computer-vision obstacle avoidance in PX4 SITL, Gazebo and ROS 2; no real flight or hardware deployment.
- Work on one visible demo at a time: M0 data, M1 empty-world A-to-B, M2 wall stop, M3 detour, M4 noise/dropout, M5 image depth.
- Current milestone: M0. Use README.md's milestone table and Backlog for v2 status until roadmap migration.
- Existing code, evidence and native roadmap are historical v1 references, not v2 prerequisites.
- Read README.md for current results. Existing files and observed runs take precedence over proposed layouts or commands.

## Working rules

- Demo first. Add guards, contracts or abstractions only for an observed failure or the current demo's concrete need.
- Minimum planned sim controls: fresh-depth watchdog (0.5 s simulation time, then zero velocity), speed clamp and explicit PX4 sim parameters.
- This documentation change does not modify runtime code or authorize control publishers, parameter writes, arming or flight.
- Simulation collisions are measured outcomes, not claims of real-world safety. Report failures and missing data honestly.
- Keep bounded work inline. No automatic review, TDD, E2E, multi-agent or self-evaluation pipeline.
- Keep PRs near 400 changed non-test code lines; split coherent work or ask before exceeding that scope.
- Reuse existing code and dependencies. Ask before adding dependencies, installing tools or changing Docker images.
- Stop after three repair attempts without new causal evidence; report the blocker.
- Do not perform future milestones while the current demo is incomplete; record deferred work in README.md Backlog.

## Review and documentation

- Triage external reviews before fixes: BLOCKS-MILESTONE, BACKLOG or REJECT, with one reason per item.
- Default nonblocking items to BACKLOG. Obtain user approval of the triage before changing code.
- Save requested external reviews under reviews/<date>.md; do not invent reviews or approvals.
- Update results in README.md. Use docs/DECISIONS.md and docs/LESSONS.md only when recording actual decisions or checked facts.
- Do not create per-iteration *_PLAN, *_RESULTS or *_CONTRACT documents or evidence bundles by reflex.
- Preserve original PDFs, raw evidence and history. Archive/reset is separate work, not permission from this file.
- Load only skills and source sections needed for the current task; no mandatory skill router or full-contract reload.

## Technical facts

- Camera optical origin in base_link FLU: (0.13233, 0, 0.02078) m.
- Depth is optical Z, not ray range. Treat invalid pixels conservatively; unknown, unseen or stale space is not free.
- Use VehicleOdometry.timestamp_sample for measurement time; convert NED/ENU and FRD/FLU explicitly. Signed components are valid.
- Use Gazebo simulation time for sim-cadenced freshness: use_sim_time=true and UXRCE_DDS_SYNCT=0; no wall-stamp substitution.
- Monotonic wall time may diagnose a stopped simulator or process; it must not replace sensor simulation timestamps.
- Preserve GZ_CONFIG_PATH=/usr/share/gz in the existing derived bridge environment.
- One final publisher owns Offboard setpoints. Heartbeat does not make an expired motion command fresh.
- Ground truth is evaluator-only, except existing explicitly documented simulated localization; never feed evaluation truth to the planner.
- COM_OBL_RC_ACT=5 requests Hold; estimator/health conditions may escalate it. Do not claim guaranteed Hold.

## M1 implementation direction

- Build a new minimal Offboard node for empty-world takeoff, A-to-B movement and landing; one node owns setpoints.
- Do not route M1 through v1 supervisor.py, px4_bridge.py, permits, Home/variance/RTF admission or prep-* gates.
- V1 code is reference material only, whether present locally or in PR #12. Reuse small useful functions, not the v1 control architecture.
- Keep the planned watchdog, speed clamp, explicit sim parameters and separate evaluator; judge M1 by its actual five-run demo.

## Verification and roadmap

- Run the smallest existing check supporting the changed behavior. Documentation edits need content/link checks, not simulator startup.
- `python scripts/check_project.py`: existing v1 document, PDF, evidence and roadmap integrity.
- `python -m pytest -q`: existing offline behavioral regressions; not flight proof.
- Until migration, do not use the native v1 roadmap to select, block or report M0–M5 work; README.md is the v2 status source.
- If the harness reads native state on startup, treat that result as historical only; do not start its focus task for v2 work.
- Keep v1 native status, receipts and portable exports unchanged. A future approved migration must use the native API, not hand-edited JSON/HTML.
- Never advertise proposed sim/up.sh, run_trials.py or slash commands as available until implemented and exercised.

## Language and authority

- Speak Vietnamese; write persisted authored text in English. Preserve verbatim sources, logs and historical evidence.
- Work on a task branch. Preserve unrelated changes; never force-push, reset/stash unknown work or bypass checks/reviews.
- Existing user authorization for requested Git work applies only to minhtri-bi/drone-CV; use concise English commits and PR bodies.
- For every new change, commit on a task branch and open a pull request for user review; never push directly to main or merge without explicit user approval of that PR.
- Confirm operational scope separately. Harness instructions and roadmap data do not authorize installation, acquisition, control or flight.
