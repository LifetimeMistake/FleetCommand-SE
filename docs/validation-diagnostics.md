# Suite v3 settling and transition diagnostics

Run `start` with the newly loaded `FleetCommand.Validation` local library
script. The LCD header must be `FCVALIDATION|1|craft-v1|suite-v3`. The blueprint
is unchanged. The programmable block's output continuously identifies
`BRAKING`, `PASSIVE REST`, or `MEASURING` and the case name, including
`thrust_py` for +Y. Copy the entire LCD text after completion or an error.

The purpose is to distinguish a settling/command transition problem from
physics behavior or stale/inconsistent API observations. This is a measurement
change, not a claim that the missing-motion discrepancy has been solved.

## Settling before every case

Active braking continues until 15 consecutive frames satisfy all of:

- Remote and cockpit linear speeds below 0.01 m/s.
- Remote angular speed below 0.01 rad/s.
- COM displacement divided by elapsed time below 0.01 m/s.
- Observed orientation change divided by elapsed time below 0.01 rad/s.

The runner then clears thrust overrides and autopilot, switches dampeners off,
and disables both gyros. It observes another 60 frames without actuator writes.
The same velocity/pose criteria must continue to hold, with total passive
displacement at most 0.01 m and rotation at most 0.01 rad. Motion restarts active
braking. The existing 900-frame total settling timeout remains; errors leave
the report on the LCD. Normal block settings are restored when the case begins.

`GATEPOLICY` records these limits. `GATECOLUMNS` describes each `GATE` row,
which records the accepted duration, stable and passive frame counts, both
controllers' speeds, pose-derived speeds, and accumulated passive drift/rotation.
Settling gate regressions inject position/orientation motion while reported
velocity is zero, disagreement from the cockpit, and motion during passive rest.

## Detailed trace on the same LCD

The full suite still produces 19 cases and 133 canonical `S` samples.
Additional `D` records focus on `thrust_px`, `coast_px`, and `thrust_py`, giving
a normally moving control case and both suspicious transitions. They include:

- The last eight active-braking frames and periodic passive-rest snapshots.
- State before the case, then the actual command written at tick zero.
- Every one of the first 12 flight frames, plus later milestones.
- For the coast pulse, the completed powered interval at tick 15, a separate
  post-write release-command row, and every frame from 16 through 27.
- COM, remote-block and PB positions, orientation quaternion, remote/cockpit
  velocities, angular speed, position-derived speed, control flags, actuator
  enabled/working masks, all six overrides and all six reported thrusts.

`DCOLUMNS` contains the complete field names. All positions and vector
velocities are transformed into the fixed initial frame; they do not require
the craft to be placed at world zero. Remote/PB positions include their fixed
mount offsets from the initial COM. Use changes in those coordinates when
comparing physical movement. Quaternions are relative to the initial attitude.
`position_speed` uses displacement since the preceding captured observation
divided by elapsed `run_tick` intervals; periodic rows therefore average a
longer interval than consecutive rows. `dt` is the current PB callback interval.

Thrust mask bit order is +X, -X, +Y, -Y, +Z, -Z. Gyro mask bits are A and B.
`flight` observations precede that callback's actuator writes and describe the
completed physics interval. `command` observations follow writes; `CurrentThrust`
may still describe the preceding interval. Command and delivered telemetry are
kept distinct deliberately. `passive_failed` records a failed coast observation.

Trace text has a 48,000-character budget within the 94,000-character report
limit. `TRACE` records actual rows, characters and dropped rows. If retries
exhaust the trace budget, canonical samples continue and truncation is explicit;
the strict comparator rejects a truncated diagnostic run. A normal native run
uses about 89 KB total, with 123 trace rows and none dropped.

The launcher still extracts the complete report through the simulated LCD API.
`run artifacts/simulator-suite-v3.txt` saves the simulator counterpart. Compare
it with a game report from suite v3. Archived v1/v2 reports remain readable,
but cross-revision comparisons fail because the settling procedure changed.

Verified locally: all 330 tests passed (293 simulator and 37 validation), MDK
packing and exact packed-source compilation passed without warnings/errors,
and the native runner completed 19 cases / 133 samples. Its final LCD export
is 88,674 characters, including 123 trace rows / 45,675 trace characters and
zero dropped rows. The verified script was copied to the game's local library.
Review identified the coast-release sampling gap and ordinary gyro damping
during passive rest; both are corrected and covered by regression assertions.
