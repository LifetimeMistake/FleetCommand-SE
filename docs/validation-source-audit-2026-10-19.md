# Source conformance audit: rotation and native physics

This is a chronological investigation record. For the final baseline, retained
game measurements, and the resolved power-update-order discrepancy, see
[station-validation.md](station-validation.md#power-discrepancy-resolved-thrust-precedes-distribution).

This pass continues the simulator work after deploying suite v4. The game script
and blueprint are unchanged. The new complete v4 game measurement is archived as
`artifacts/game-suite-v4-2026-10-19.txt`; the earlier v3 measurement remains archived
separately. Neither raw measurement has been rewritten to match the simulator.

## Implemented source paths

`AutopilotSystem.ComputeTurnTorque` now follows the ordinary vacuum, default
Forward path in `MyAutopilotComponent.UpdateGyro` and `GetAngleVelocity`:
remote-mounted orientation, quaternion aim error, adaptive angular rate, and
the stopping test using predicted angular velocity. The inherited
`RotationDamping = 0.5` approximation is removed. Opposite headings no longer
silently return zero because a cross product vanishes.

The command remains at the source rate (up to 2 rad/s under the current default
definition values). The actual gyro torque application clamps components later;
there is no additional unit-sphere clamp in the autopilot decision. The source's
explicit NaN stopping-ratio branch is retained, including pure-axis zero/zero
components; it is not replaced with a guessed scalar stopping rule.

`GyroRateSystem.PredictAngularVelocity` ports `MyGridGyroSystem.GetAngularVelocity`:
inverse inertia, force/override authority, optional smoothing, power gating,
world/grid transforms, and the native active-body tiny-velocity condition.
Predictions do not apply an impulse. This host refreshes a dirty parameter cache
at the query boundary; the game maintains that cache through scheduled component
updates. The host's power-filtered cache also invalidates on working-state
transitions, so restored supply cannot temporarily predict zero authority.

Gyro power, functionality, enablement and autopilot changes invalidate authority.
An identical override flag no longer restarts the acceleration ramp, following
`Sync<bool>.SetValue`. This does not suppress equal rate writes through
`SetGyroTorque`, which unconditionally marks the source gyro system dirty.
Powered nonzero overrides disable native automatic deactivation, and releasing
the override restores it, following `UpdateAutomaticDeactivation`.

`SimWorld` now constructs an explicit `HkWorld.CInfo` from
`MyPhysics.CreateWorldCInfo`, including solver tau/damping, two solver microsteps,
island sizing, collision settings and expected timestep. World rotation
deactivation thresholds are divided by three, as in `CreateHkWorld`. The native
convenience constructor previously delegated to opaque native code. This removes
that unverified configuration dependency; it is not evidence that every former
native setting differed.

Thruster functionality transitions now wake native physics even if the thruster
is disabled and `IsWorking` remains false. This follows
`MyThrusterBlockThrustComponent.ComponentStack_IsFunctionalChanged`, rather than
using a working-state event as a proxy. Dampener transitions also wake sleeping
grids, following `MyCubeGrid.EnableDampingInternal`; identical writes and grids
without thrusters do not wake. The latter guard was added after review reproduced
the missing-component edge case. The host still stores a shared dampener flag on
controller-only grids; fully reproducing absent thrust-component getters and
setters remains outside this wake correction.

## Evidence and limits

- Three rotation regressions failed before the source replacement.
- Solid-cube inertia/power predictions match independently calculated values.
- Functional damage and actual battery loss/restoration invalidate cached
  authority before the next actuator aggregation; both defects were reproduced.
- Repeated override writes preserve the source ramp countdown; the old setter
  restarted it.
- Native sleep state follows powered override changes.
- Seven additional native sleep regressions cover functionality changes in both
  directions, dampener transitions and equal writes, and absent thrust components.
  The missing wake paths failed before their fixes; the absent-component test
  caught the initial correction's extra wake before its guard was added.
- Before the PB scheduling pass, all 321 simulator tests passed. Validation passed 36 of 37 tests.
  `RotatedRun_NormalizesTheEntireSuite` still fails on waypoint velocity/thrust
  samples. Its assertions and game-comparison tolerances remain strict.

The shared runner still completes 19 cases / 133 samples and all 88 waypoint
diagnostic rows. Its current source-audit LCD export is
`artifacts/simulator-suite-v4-wake-audit.txt` (55,693 characters), retained
separately from earlier measurements. `artifacts/source-conformance-wake-tests.log`
contains the full test result. The simulator has not been declared equivalent
to the game.

The existing game trace shows full thrust telemetry while the first three
velocity increments differ from native simulation. Explicit world configuration
did not remove this difference. No artificial force ramp or timing delay was
introduced. The source's thrust scheduling and resource-distribution ordering
remain investigation targets, along with the pending v4 waypoint trace.
Specifically, the grid queue schedules thrust at priority 6, gyro updates at 7,
and the resource distributor at 14. `Sink_RequiredInputChanged` marks distribution
dirty rather than always recomputing it synchronously. This differs from the
host's current allocation-before-actuation pipeline. A scheduling repair must
also account for synchronous source-change recomputations and parallel callbacks;
no blanket one-frame delay was added based solely on the numerical discrepancy.

The host still uses a single-threaded continuous Havok world. This is not an
implementation of the game's parallel update scheduling. Planetary alignment,
advanced AI behaviors and nondefault remote flight directions remain outside the
ordinary rotation path implemented here.

## New v4 game measurement and PB update phase

The new game export contains all 19 cases, 133 canonical samples, 123 diagnostic
rows and 88 waypoint rows, with no dropped rows. Its normalized length is 61,895
characters. The new trace records no motion or thrust at waypoint ticks 1
and 2; thrust/motion first appear at tick 3. The previous simulator first responds
at tick 1. This is measured latency evidence, not a justification to insert a
fixed two-frame delay.

Comparing displacement from each case's own first sample separates physical
response from the accumulated effects of earlier cases and settling duration:

| Case | Simulator displacement (m) | Game displacement (m) | Simulator/game final speed (m/s) |
| --- | ---: | ---: | ---: |
| +X thrust | 2.168985 | 2.157073 | 2.860243 / 2.851831 |
| +Y thrust | 2.168985 | 2.157068 | 2.860244 / 2.851818 |
| +Z thrust | 2.168985 | 2.157075 | 2.860244 / 2.851826 |
| Battery discharge | 4.782899 | 0.007487 | 1.906834 / 0 |
| Waypoint | 27.383313 | 27.280447 | 2.224001 / 2.345696 |

Negative-axis thrust displacement is likewise about 2.15707 m in game versus
2.16899 m in the simulator. The battery-only stop is reproducible in this fresh
measurement while telemetry continues to report 2,880 N. Its cause remains open.
`artifacts/suite-v4-case-relative-summary.csv` retains all case summaries; these
derived summaries do not replace or loosen the canonical report comparison.

Source inspection confirmed a distinct scheduling defect:
`MyIngameScriptComponent.UpdateBeforeSimulation/10/100` latches invocation flags,
but `UpdateAfterSimulation` actually executes the PB. The launcher had manually
executed PB code through the host's before-physics scenario callback. The host now
latches scheduled sources before physics and invokes PB code after the completed
interval and clock advancement. `Runtime.UpdateFrequency` now schedules it;
the launcher no longer duplicates scheduling through `EveryUpdate1`.
The existing scenario-input hook remains before physics.

Regression coverage checks completed-state observations, next-interval thrust,
Once rescheduling, combined update flags, empty scheduled arguments, stopping,
and disabled/nonfunctional/unpowered host execution. Review caught a missing
working-state gate in the new host entry; it was reproduced and corrected.
Cadence currently uses the host's global tick phase, rather than the game's
per-entity staggering. This is not a port of remote/grid parallel scheduling or
resource priorities. It does not resolve the waypoint startup latency or battery
stop by itself. No artificial actuator delay was introduced.

An isolated native body probe using the same mass and constant 2,880 N remains
awake and accelerates under both Continuous and Multithreaded simulation types
when stepped serially. That rules out changing only the world simulation-type
enum as a demonstrated fix; it does not test a fully multithreaded solver or the
complete ship/component lifecycle. Probe outputs are retained in
`artifacts/havok-sleep-continuous.txt` and `artifacts/havok-sleep-mt.txt`.

The post-PB-phase report is `artifacts/simulator-suite-v4-after-pb-phase.txt`
(55,655 characters; complete 19/133). The strict game comparison still fails,
retained in `artifacts/suite-v4-after-pb-phase-comparison.txt`.
Final native verification passes all 328 simulator tests and 36/37 validation
tests; `RotatedRun_NormalizesTheEntireSuite` remains the one validation failure.
The full output is `artifacts/pb-after-simulation-tests.log`. The shared game
script has not changed during this scheduling repair, so no replacement script
or new game measurement is required yet.

## Manual moving-start check and native isolation

The operator subsequently reproduced the low-thrust stall manually in game.
Starting near rest with 2,880 N barely moves the grid; starting at approximately
0.99 m/s and applying the same override for roughly five seconds reaches about
2.97 m/s. These are hand measurements, not canonical samples or new tolerances.
The 1.98 m/s increase corresponds to about 5.19 seconds at nominal acceleration
(`2880 / 7551.798828` m/s²). This supports a rest-dependent mechanism rather than
a constant loss of force. It does not establish a particular speed cutoff or
prove automatic deactivation is the cause.

Expanded native probes retain full output under `artifacts/havok-sleep-*.txt`:

| Native configuration | Initial speed (m/s) | Speed after 300 frames (m/s) | Stall |
| --- | ---: | ---: | --- |
| Frame notifications + serial step | 0 | 1.906834 | No |
| Frame notifications + serial step | 0.99 | 2.896819 | No |
| Frame notifications + 8,192 m local offset | 0 | 1.906834 | No |
| Actual thread pool + multithreaded StepSimulation | 0 | 1.906834 | No |
| Actual thread pool + InitMtStep/FinishMtStep job queue | 0 | 1.906834 | No |
| Actual thread pool + InitMtStep/FinishMtStep job queue | 0.99 | 2.896819 | No |

The job-queue path follows `MyPhysics.StepWorldsParallel`, enabled by the stock
`ENABLE_HAVOK_MULTITHREADING` and `ENABLE_HAVOK_PARALLEL_SCHEDULING` defaults.
The frame notifications follow `MyPhysics.SimulateInternal`. These are isolated
box-body probes, not a reproduction of the complete grid shape and lifecycle.
No production solver setting was changed on the basis of these results.

Read-only inspection of the validation session's saved configuration confirms
offline Survival, no world mods, eight physics iterations, and selective physics
updates disabled. However, the session log records Pulsar loading a preprocessed
Sandbox.Game assembly under .NET 10 and the Performance plugin. The simulator
loads stock assemblies under .NET Framework 4.8. The preprocessed HavokWrapper
hash also differs from stock. This is an environment difference, not proof of
causation. The inspected Performance `MyPhysicsBodyPatch.RigidBodyGetterTranspiler`
rewrites body lookup; it does not provide an observed low-speed braking rule.
Installed IL dumps are retained as ignored diagnostic artifacts, not checked-in
game/plugin binaries or copied into simulator source.

A stock-game manual rest-start comparison has been requested to isolate that
environment difference. No guessed velocity threshold, drag term, force ramp,
or sleep timer has been added to the simulator. The latest canonical game report
and its measured discrepancies remain unchanged.

The operator then disabled the Performance plugin and still reproduced the same
stall. This reduces that plugin's likelihood as the cause; it does not establish
a full stock-runtime comparison. No further plugin toggle is required for the
next diagnostic. A native HkGridShape probe (same box child, grid root) also
accelerates normally from rest and from0.99 m/s; outputs are retained as
`artifacts/havok-sleep-grid-rest.txt` and `artifacts/havok-sleep-grid-moving.txt`.

The shared runner now includes a separate `restprobe` command to compare fixed
20% thrust, explicit grid wake, override refresh, and a moving-start control.
This reuses passive settling and relative-frame LCD reporting, without changing
the canonical v4 experiment or any simulator force law. Details and operation:
[low-speed diagnostic](validation-low-speed-diagnostic.md). Native output is
`artifacts/simulator-low-speed-v1.txt` (37,226 characters; complete4/265).
The packed script was compiled under the actual PB wrapper and deployed with
SHA256 verification. All328 simulator tests pass; validation passes38/39 tests,
with the same existing rotated-waypoint failure. A test-host startup failed once
with an out-of-memory exception before validation execution; the separate retry
completed (`artifacts/low-speed-validation-final-tests.log`). This diagnostic
is ready for game measurement; the low-speed mechanism is not yet identified.

## Low-speed game measurement and corrected moving control

The complete v1 game report is archived as
`artifacts/game-low-speed-v1-2026-10-19.txt`, SHA256
`71AF6660F2D880C3C14F5F3C683E1140308720C48186E6686F22BF1C59D495FB`.
With fixed 20% override, `low_hold` initially approaches 0.011375 m/s and then
stops, covering only 0.00749 m. Both explicit wake and override refresh sustain
about 0.011375 m/s, covering about 0.073942 m during their measured intervals.
Telemetry reports 2880 N, reactors off and dampeners off. These interventions
prevent the complete stop but do not restore nominal acceleration; the result
does not establish a force law or a native sleep threshold.

The operator correctly identified that v1's moving control never became a valid
moving start. Full-thrust warmup reaches 0.183010 m/s at tick 6, reports zero
at tick 7 despite 14400 N telemetry, and remains stationary at the switch to
20%. The diagnostic now uses `low-speed-v2`: warmup must reach a measured
1 m/s along the active thruster's world-space thrust direction, with full
override refreshed and the grid explicitly woken during warmup only. It then
starts a separate 300-frame reduced-thrust measurement without either
intervention. A 600-frame timeout reports an error before creating that case.
Warmup telemetry and its completion speed are included on the LCD. Canonical
suite-v4 and simulator physics are unchanged.

Native v2 output is `artifacts/simulator-low-speed-v2.txt` (34,515 characters,
complete 4/232). Warmup takes 32 frames and reaches 1.016977 m/s along thrust;
the reduced-thrust measurement ends at about 2.923815 m/s. Regression tests
cover measured-speed gating and explicit failure when velocity cannot build.
All 328 simulator tests pass; validation passes 40/41, with the existing strict
rotated-waypoint failure still present. The packed script compiles under the
game PB wrapper and was deployed with SHA256 verification. No guessed
low-speed damping, force ramp or velocity threshold was added to physics.

## Valid moving-start game result

The complete v2 game report is archived as
`artifacts/game-low-speed-v2-2026-10-19.txt`, SHA256
`06DFCA31DC27D9BF47F75AC687F986575B01A7A3242502823BAF19F7ACB915BD`.
Warmup reaches 1.009086 m/s in 32 frames. During the following five seconds at
2880 N, velocity rises to 2.915382 m/s: a gain of 1.906296 m/s, versus
1.906830 m/s from force divided by the reported 7551.798828 kg mass and elapsed
simulation time. The difference is about 0.028%. The simulator's corresponding
gain is 1.906839 m/s. Absolute final speeds must not be compared without
accounting for the different measured entry speeds.

The other three cases repeat the v1 behavior: fixed override travels only
0.00749 m and stops; wake and refresh each sustain about 0.011375 m/s and travel
0.073942 m. All four cases use the same reduced override and battery-only
measurement configuration. This argues against a general thrust calibration or
battery-power deficit as the explanation for the rest-start discrepancy. It
isolates the remaining difference to low-speed behavior without establishing
its implementation or an exact transition threshold. Physics remains unchanged
pending a source-backed explanation.

## Further deactivation isolation

The managed force path in `MyPhysicsBody.AddForceInternal` rejects an
`activeOnly` force after a body sleeps. `MyThrusterBlockThrustComponent`
queues thrust with that flag. This accounts for persistence of a stopped body,
but not the preceding reduced acceleration. Default grid linear damping is zero;
the explicit managed clamp in this path limits maximum speed rather than
implementing low-speed braking. The inspected grid deformation slowdown requires
contact/deformation and does not establish a cause in the empty fixture.

Additional native probes use the same 7551.8 kg body, proper box inertia and a
grid root. Low, Medium, High, Max and Off solver deactivation classes, disabled
body deactivation, and repeated wake all give about 1.906834 m/s after five
seconds at 2880 N from rest. The ordinary classes allow the body to sleep during
the preceding unforced settling period, confirming that sleep is enabled in
those probes. Logs are `artifacts/havok-deactivation-*.txt`.

The same probe also accelerates normally under .NET 10.0.8, both with stock
HavokWrapper and the game's Pulsar-preprocessed wrapper. This does not reproduce
the whole game environment, but those substitutions alone do not cause the
stall (`artifacts/havok-net10-rest.txt` and
`artifacts/havok-net10-pulsar-wrapper-rest.txt`). Native inspection of the
PlanetGravityAction application function shows it returns without applying
force when its gravity vector is zero. Disassembly artifacts are ignored, and
no native machine code is copied into production source.

The next game measurement uses `low-speed-v3`, retaining the four v2 cases and
adding two gyro controls. `MyGridGyroSystem.UpdateAutomaticDeactivation` permits
automatic deactivation for a zero target and disables it for a powered nonzero
target. The new controls enable one gyro with either zero rate or 0.01 rad/s
about the thrust axis, set once at case entry. Both use the same five seconds of
battery-only 20% thrust; there are no repeated explicit wake or refresh writes.
Extra LCD rows expose the gyro commands, working state and actual thrust axis.
The distinction is conditional on the gyro remaining powered and is not a direct
measurement of the game's inaccessible native flags.

Two new rotated-fixture regressions first failed because the cases did not
exist, then passed after implementation. They verify the actual native
`EnableDeactivation` flag, command alignment and uncurtailed 2880 N thrust.
Native v3 LCD output is `artifacts/simulator-low-speed-v3.txt` (56,994 characters,
complete 6/348). The root cause of the game-only low-speed suppression remains
unidentified; simulator physics and canonical suite-v4 are unchanged.

Verification for this extension: all 328 simulator tests pass; validation passes
42/43 tests, with only the existing strict rotated-waypoint failure. A scoped
independent review found no actionable measurement or implementation issues.
The packed script compiles under the actual game PB wrapper with no warnings
or errors and was deployed with matching SHA256
`B3AF356756855F3FF62A93E15D6711A401DA78889A568FE07ABEBC9E23D39C7D`.

## Diagnostic report headroom correction

The operator reported a v3 overflow in the game. The identity-run size check
left too little margin for real game telemetry; a rotated native run alone did
not reproduce the overflow, but also remained about 57 KB. Version
`low-speed-v4` retains all six 300-frame experiments and unchanged physics
commands, while recording initial state, ticks 1 through 12, and every 60th tick
through tick 300. This gives 18 full samples per case (108 total). Gyro settings
and actual thrust direction are recorded at ticks 0, 10 and 300, rather than
repeating them for every sample. Warmup success, timeout and telemetry remain.

The complete native LCD report is now 19,485 characters
(`artifacts/simulator-low-speed-v4.txt`). Two new regressions first failed on
v3's insufficient headroom, then passed after compaction. They check completion
and a 35,000-character ceiling under identity and rotated placement, and stress
continuous numeric fields with full-width signed six-decimal values plus the
maximum warmup row count. That conservative estimate must remain at least
2,000 characters below the report's 64,000 limit. All eight focused diagnostic
tests pass. The first 12 ticks retain the observed tick-7 stop, and per-second
samples retain the sustained creeping versus normal acceleration comparison.

## Game v4 gyro control and native solver reproduction

The complete game report is archived as
`artifacts/game-low-speed-v4-2026-10-19.txt`, SHA256
`586172D0B60985FC3E0FE2AB2386DBE062E76FF066979BC2FD23DA176B7AE747`.
It ends complete 6/108 without overflow. `low_gyro_zero` sustains only
0.011375 m/s, covering 0.073942 m. `low_gyro_spin` reaches 1.864452 m/s and
covers 4.577723 m in five seconds. Both report 2880 N, battery-only operation,
working gyro override and dampeners off. The spin target is 0.01 rad/s and its
logged thrust direction stays effectively constant. Its initial acceleration
is still suppressed for several ticks before approaching the nominal value.
The valid moving-start case repeats about 1.906295 m/s gain in five seconds.

Native investigation now reproduces the creeping by making the existing
box/grid-root probe participate in a fixed constraint with a second identical
dynamic body, ten metres away. Both bodies receive the same per-mass thrust;
the constraint is initially satisfied. This selects a native constraint-solver
path absent from the earlier isolated-body probe. It does not prove that the
game fixture has a hidden constraint or establish why its body uses that path.

| Native condition | First-tick speed (m/s) | Speed after five seconds (m/s) |
| --- | ---: | ---: |
| Isolated body, Low solver deactivation | 0.006356101 | 1.906834 |
| Fixed pair, Low solver deactivation | 0.004077997 | 0.01134793 |
| Fixed pair, automatic sleep disabled on both bodies | 0.004077997 | 0.01137794 |
| Fixed pair, automatic sleep disabled on driven body only | 0.004077997 | 0.01137794 |
| Fixed pair, Off solver deactivation | 0.006356101 | 1.906857 |
| Fixed pair, both bodies initially moving at 0.99 m/s | 0.9963565 | 2.896869 |
| Pair after removing its fixed constraint before thrust | 0.006356101 | 1.906834 |

The creeping also reproduces with the parallel job-queue stepper. Merely adding
an unconnected second body does not reproduce it. Explicit sphere-inertia and
thin-box-inertia motion types still accelerate normally when isolated. Logs are
`artifacts/havok-island-*.txt`, `artifacts/havok-solver-path-*.txt`, and
`artifacts/havok-motion-*.txt`.

This separates automatic sleeping (`EnableDeactivation`) from solver
deactivation/stabilization (`Motion.GetDeactivationClass`). Disabling sleep
does not remove the native low-speed suppression; changing the solver class to
Off does. Therefore the game gyro result cannot be attributed solely to the
stay-awake flag: the induced angular motion is another relevant change.
These controls identify a native mechanism capable of the measured behavior,
without justifying an empirical drag term or an artificial constraint in the
simulator. The remaining work is to trace that solver path and explain why
the game grid takes it while the simulator's unconstrained grid does not.

Native disassembly further locates an accumulator-processing branch at Havok
RVA `0x62C870` (this installed binary's preferred image address
`0x18062C870`). It reads the accumulator's solver deactivation class, selects a
32-byte parameter record, combines componentwise absolute linear and angular
velocity with class-dependent weights, and conditionally multiplies both
velocity vectors by a damping parameter or sets both to zero. It is called
from the native solver iteration loop. This is direct binary evidence for a
small-motion suppression mechanism separate from the managed maximum-speed
clamp and ordinary grid linear damping. The artifact is
`artifacts/havok-solver-low-speed-damping.txt`; it is diagnostic disassembly,
not a portable API or an implementation to copy verbatim into the simulator.

Read-only inspection of the isolated world's configuration is retained in
`artifacts/havok-solver-native-config.txt`. The native rotation-threshold getter
offsets were checked against their exported values. The observed record for
the Low class includes weights 10.19368 and 10.39111, damping 0.984, and a
zeroing threshold 0.025. These observations are not an empirically fitted
production force law; accumulator processing and solver iteration timing still
matter. Directly enabling the source grid breakability hook or contact callback
settings on an otherwise isolated body does not reproduce the suppression
(`artifacts/havok-grid-hook-*.txt`). No new simulator setting, artificial
constraint, or diagnostic script deployment follows from this measurement.

## Resolution: the connector's own detector constraints

The missing constraints are not docking connections. `MyShipConnector.LoadDummies`
creates both a spherical connector detector and a box ejector detector. Each is
a dynamic phantom body with the native default mass (1 kg), fixed to the grid
by `CreateBodyConstraint`, using stabilized solving and inertia stabilization 1.
They exist unpaired and while the connector is disabled. The user independently
confirmed that disabling the connector retains the low-thrust stall while
removing the block removes it.

The simulator now creates these source bodies, using dummy matrices extracted
from the installed 1.210.014 models for the five supported connector subtypes.
It preserves the source's final activated body pose, block-local constraint
pivots, collision filter, default mass/inertia, damping and speed flags. These
two auxiliary masses are excluded from `CalculateShipMass`, as in the game.
Teleports synchronize their poses and point velocities; removal and teardown
release constraints before bodies. No fitted drag, velocity cutoff or force
threshold was introduced. Native Havok supplies the low-speed behavior.

This exposed a missing source guard in manual gyros: `UpdateGyros` returns before
actuation when angular velocity and control torque are componentwise below
0.001. Without that guard, numerical angular noise repeatedly woke the grid.
The guard now follows the source. Gyros also now read the actual native inverse
inertia tensor instead of the inverse of our analytical full mass tensor.
Havok's effective diagonal differs by approximately 2% on this fixture; using
the analytical inverse was a real mismatch with `MyGridGyroSystem`.

## Gyro interaction and world orientation

After those repairs, the apparently remaining gyro-under-thrust discrepancy
isolated to initial world attitude. The exact same shared `restprobe` script,
fixture and commands produce these native results:

| Quantity | Recorded game v4 | Simulator identity attitude | Simulator yaw .7, pitch -.3, roll 1.2 |
| --- | ---: | ---: | ---: |
| `low_hold` displacement (m) | 0.007490 | 0.007488 | 0.007488 |
| `low_wake` final speed along +X (m/s) | 0.011375 | 0.011375 | 0.011375 |
| `low_moving` velocity gain (m/s) | 1.906295 | 1.906287 | 1.906200 |
| `low_gyro_zero` displacement (m) | 0.073942 | 0.072449 | 0.072113 |
| `low_gyro_spin` displacement (m) | 4.577723 | 4.577746 | 0.073820 |
| `low_gyro_spin` final speed (m/s) | 1.864452 | 1.864462 | 0.011378 |
| `low_gyro_spin` final angular rate (rad/s) | 0.009983 | 0.009983 | 0.006405 |

The first identity-attitude gyro samples also match the recorded game to six
decimals: tick 1 angular rate 0.006402, tick 3 rate 0.007100, tick 10 rate
0.009571 and speed 0.035415. This is the unchanged maneuver through the shared
runner, not an alternative scripted controller. The existing native accumulator
disassembly explains why rotations cannot simply be normalized away: the
suppression branch combines weighted absolute velocity components and makes
componentwise comparisons. It is not a rotationally invariant drag law.

The v4 game report did not record absolute initial attitude. The close identity
match is strong evidence, not proof of the exact original game world matrix.
An updated game report is needed to replay its actual attitude and confirm the
orientation dependence directly. Broader autopilot equivalence remains open;
these low-speed results do not establish equivalence of every framework feature.

## Orientation-aware report and verification

Suite v5 / low-speed-v5 retain the v4 maneuvers and samples and add mandatory
`META|initial_qx`, `initial_qy`, `initial_qz`, `initial_qw` fields. All position,
velocity and attitude samples remain relative to the original frame. The launcher
accepts `--frame-from game-report.txt`, maps the recorded reference-controller
attitude through its blueprint mounting orientation, and runs the same script.
The strict canonical comparator rejects differing initial world attitudes;
measurement tolerances are unchanged. Legacy v1-v4 schemas remain readable,
but current v5 reports with missing frame metadata fail validation.

The old whole-suite rotation-invariance assertion was replaced with an explicit
incompatible-attitude assertion because native physics disproves that premise.
Whole-suite translation invariance still passes, and both orientation-dependent
gyro responses have regression coverage. Additional coverage verifies native
inertia for asymmetric mass distributions, offset-COM/rotated-block teleports,
direct connector/grid disposal, and disabled-versus-removed constraints.

Verification: 335 simulator tests and 55 validation tests pass in
`artifacts/gyro-connector-final-tests.log`. Full native LCD exports are
`artifacts/simulator-low-speed-v5-identity.txt` (22,111 characters),
`artifacts/simulator-low-speed-v5-rotated.txt` (22,978),
`artifacts/simulator-suite-v5-identity.txt` (62,187), and
`artifacts/simulator-suite-v5-rotated.txt` (62,151). All have complete footers
and are below the 65,000-character publication cap. Independent scoped review
found a missing-frame validation gap; a failing test reproduced it before the
report revision and mandatory-field fix.

### Replay precision and remaining waypoint limitation

Initial quaternion metadata uses round-trip formatting, without CultureInfo in
the PB. Ordinary measurement rows retain their six-decimal representation.
The pose audit also found a real source-boundary mismatch: the simulator rebuilt
each grid world matrix from the native quaternion getter, while
`MyPhysicsBody.GetWorldMatrix` reads the native rigid-body matrix directly.
The simulator now caches that actual matrix at creation, state replacement and
post-physics synchronization. A failing native-matrix regression established
the discrepancy before the correction.

World-attitude replay is not a complete Havok checkpoint. Matrix/quaternion
conversion and normalization can change float bits; the report records the
attitude after startup, and does not restore detector motion or native solver
state. A proposed whole-suite replay-equality assertion was investigated and
rejected as an unsupported invariant, rather than widening tolerances to make
it pass. The actual strict CLI comparison remains available in
`artifacts/gyro-frame-cli-comparison.log`: all non-waypoint channels pass, but
the approximate waypoint controller still changes sampled thrust direction and
some velocities between near-identical initial states. This limitation remains
open. Neither passing unit tests nor the close gyro match establishes complete
game equivalence. Further waypoint work should locate the first divergent
control decision against source and a game report at a recorded attitude.

## Fresh v5 game confirmation at its recorded attitude

The updated game report is archived at
`artifacts/game-low-speed-v5-2026-10-19.txt`, SHA256
`A245CE5B0FDB3AE22E33D442E0F2D4DB122B095E957495EE30FF2DF9DF164CFE`.
It has all six cases / 108 samples and a complete footer. The shared launcher
replayed its actual initial reference orientation with `--frame-from`, producing
`artifacts/simulator-low-speed-v5-game-frame.txt` (22,558 characters).

| Quantity | Game | Simulator at recorded attitude |
| --- | ---: | ---: |
| `low_hold` displacement (m) | 0.007488 | 0.007488 |
| `low_wake` displacement (m) | 0.073940 | 0.073912 |
| `low_refresh` displacement (m) | 0.073939 | 0.073912 |
| `low_moving` velocity gain (m/s) | 1.906273 | 1.906298 |
| `low_gyro_zero` displacement (m) | 0.073940 | 0.072449 |
| `low_gyro_spin` displacement (m) | 4.577797 | 4.577710 |
| `low_gyro_spin` final speed (m/s) | 1.864491 | 1.864450 |
| `low_gyro_spin` final angular rate (rad/s) | 0.009983 | 0.009983 |

The largest endpoint displacement difference is approximately 1.5 mm in the
zero-target gyro case. The nonzero gyro case's final speed differs by
0.000041 m/s. Its first-tick and tick-3 angular rates match to six decimals.
This confirms the repaired low-speed behavior for the actual measured fixture
and attitude; it does not establish every orientation or operating regime.
No additional simulator constants or tolerances were changed for this replay.

Practical remaining priorities are waypoint steering/braking, power-starved
acceleration versus reported thrust, and paired connector capture/release before
docking automation relies on it. The existing limited-power maneuver drives
opposing thrusters together, so force telemetry alone cannot establish their
actual available net acceleration. Other unmeasured surfaces can follow the
fleet feature work that actually depends on them.

## Expanded station mission

The follow-up implementation is documented in [station-validation.md](station-validation.md).
One assembly blueprint contains a static large-grid station and an initially
connected small-grid craft. Both hosts run the shared `dockprobe` mission and
its nested `flightprobe`; complete observations remain on separate LCDs.
This adds the requested measurements without declaring the underlying power
shortage or waypoint differences fixed.

Connector source review removed automatic proximity locking, corrected imported
mating offsets/radii from model dummies and local-first rotation composition,
classified connector compatibility by port size rather than grid size, and
matched source maintenance/detection cadence. Serialized master/slave links are
restored as initial scenario state; running scripts never teleport. SBC metadata
round trips retain grid pose/static flags and paired entity IDs.

Native mission completion, two world orientations, abort, reload during retreat,
capture loss/reacquisition and unique blueprint IDs are covered. Fresh full-suite
verification passes 349 simulator and 64 validation tests. The packed PB wrapper
compiles. Reports are `artifacts/station-docking-simulator.txt` and
`artifacts/station-docking-simulator.flight.txt`. Actual game paste/connection
restoration and mission execution still need measurement. Per-grid resource and
terminal models do not become complete connected-construct models merely because
native physical connector constraints exist.

The completed game station reports and subsequent source-confirmed correction
to thrust/power update order are recorded in
[station-validation.md](station-validation.md#power-discrepancy-resolved-thrust-precedes-distribution).
The limited-power velocity gains now match within 0.000024 m/s from rest and
0.000005 m/s while moving. The corrected native mission also returns and locks.
