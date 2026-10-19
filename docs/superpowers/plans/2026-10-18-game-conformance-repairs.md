# Game conformance repairs implementation plan

> **For agentic workers:** Use superpowers:executing-plans for native execution, as requested by the user.

**Goal:** Correct simulator defects demonstrated by `TestResults/result.log` and verified against local decompiled game source.

**Architecture:** Keep the MDK script as the shared measurement source. Fix simulator subsystems at their source; preserve the original game report and strict comparison tolerances. Separate established defects from measurements that still need investigation.

**Tech stack:** C# / net48, NUnit, Havok, MDK2.

**Spec:** `docs/superpowers/specs/2026-10-18-cross-environment-validation-design.md` and user's survival / empty-world measurement context.

## Constraints and review focus

- No automated AppData reads. No tolerance changes to conceal differences.
- Thruster offsets must not introduce torque, including rotated grids.
- Unoccupied dampeners must operate; stale pilot input must not.
- Terminal queries must exclude armor while preserving physics block membership.
- Single-waypoint autopilot must require OneWay mode.
- Idle and overridden thrust must request appropriate power; depleted supply must remain bounded.

## Tasks

- [x] Add regression for off-centre thrust with zero angular acceleration; observe failure, apply resultant linear impulse in ForceAggregator; correct misleading source comments; run thruster tests.
- [x] Add unoccupied dampener regression; observe failure, separate dampening from pilot occupancy in BaseControlSystem, verify control tests and source ownership of dampener state.
- [x] Add armor exclusion regression covering terminal queries; observe failure, retain structural blocks for grid geometry while excluding from terminal APIs; run hierarchy tests.
- [x] Add single-waypoint Patrol rejection / OneWay acceptance regressions; observe failures, implement FlightMode/gating in SimRemoteControl and configure shared waypoint test explicitly; update existing autopilot scenarios.
- [x] Trace dynamic thrust demand and producer priority in game source, add idle/override regressions, observe failures, correct SimThrust/SimPowerBus and test related scenarios.
- [x] Investigate fixture mass and intermittent stationary thrust using source and measured values. Implement only established causes; document open questions.
- [x] Run complete solution tests, packed-script compile, launcher export and original-report comparison. Document remaining differences and provide updated game script when a new measurement is useful.

## Execution evidence

- Native source investigation and regressions reproduced each diagnosed defect before correction.
- Ruling: preserve synthetic one-step battery equation fixtures, while imported game definitions use100-frame energy updates and0.8 recharge multiplier. The fixture path now models game scheduling without rewriting unrelated analytic fixtures.
- Ruling: suite-v2 explicitly selects OneWay; preserve v1 parser support and reject cross-revision comparisons instead of treating changed commands as equivalent.
- Independent final review found a group-idle-floor issue; corrected after reproducing it in four throttle cases. No Critical/Important findings remained in supported power accounting/control ordering.
- Full solution:288 simulator tests +30 validation tests passed (318 total),0 failures.
- MDK build and exact packed-source compile:0 errors,0 warnings.
- Native launcher exported complete suite-v2 LCD report:19 cases,133 samples,40189 characters. Report parses and self-comparison passes; this does not establish game equivalence.
- Verified deployment to the game's local script library; existing blueprint files unchanged.
- Original game log preserved, SHA256 C1A786C6A8363627A2D811CDF323A725931A29EE311AAC78AFE43458E1E2BF15.
- Next external evidence: fresh suite-v2 game run. Unexplained old motion/settling measurements are documented, with no tolerance relaxation.