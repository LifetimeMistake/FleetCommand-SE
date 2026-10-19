# Suite v2 game comparison

Historical investigation record, including subsequent suite revisions. The
current verified baseline and retained game reports are documented in
[station-validation.md](station-validation.md#power-discrepancy-resolved-thrust-precedes-distribution).

The updated `TestResults/result.log` completes 19 cases / 133 samples with
`FCVALIDATION|1|craft-v1|suite-v2`. Autopilot now enables successfully. Game
mass 7551.798340 kg agrees with simulator 7551.800293 kg; both expose 17 terminal
blocks. Idle/charging/discharge power matches closely, and the individual gyro
rates agree closely. The strict comparison still fails; tolerances are unchanged.

## Directional movement recorded by the game

Distances below are the magnitude of the COM displacement between each case's
first and last sample, in the relative initial frame. They do not include the
settling movement between cases.

| Case | Game displacement (m) | Game final speed (m/s) |
| --- | ---: | ---: |
| thrust_px |2.157076|2.851830|
| thrust_nx |2.157070|2.851825|
| thrust_py |0.007414|0.000000|
| thrust_ny |2.157067|2.851828|
| thrust_pz |2.157077|2.851831|
| thrust_nz |2.157073|2.851826|
| coast_px |0.007414|0.000000|

The +Y case **does report 14400 N** throughout its powered samples. Its recorded
movement is the anomaly, not missing thrust telemetry. It runs at approximately
12.12–13.62 seconds after script start. This conflicts with the user's visual
observation of similar movement on every axis; the log alone does not establish
that the craft physically stayed still. Telemetry and physical behavior remain
separate questions. No accidental-contact explanation is assumed.

## Additional source-backed simulator repairs

`MyThrusterBlockThrustComponent.cs:201` supplies a COM position and
`activeOnly:true` to `MyPhysicsBody.AddForce`. Because position is supplied,
`AddForceTorqueBody` selects Havok `ApplyForce` at that point rather than its
positionless `ApplyLinearImpulse` branch. The simulator now follows that path
and skips zero thrust instead of repeatedly applying a zero impulse that wakes
the body. Ongoing force does not wake sleeping bodies; changed override values
and enabled state activate the body as the game's dirty-parameter path does.
Pilot input also activates a sleeping grid, following `MyShipController`.

Body construction explicitly uses `HkSolverDeactivation.Low`, except dynamic
large grids use `Off`, following `MyPhysicsBody` and `MyGridPhysics.CreateBody`.
The solver setting is distinct from a guarantee that a body never deactivates.
Regressions check those settings, natural sleeping under zero thrust, ignored
force on sleeping bodies, and waking from valid pilot input.

These repairs do **not** explain the original missing-motion samples: the
repaired simulator still moves in +Y and during the coast pulse. Game settling
is also often shorter. The simulator's approximate waypoint behavior remains
different from the game, and stored-battery-energy samples remain sensitive to
the independent 100-frame update phase. Limited-power game telemetry reports
full thrust while the simulator scales delivered thrust; that remains open.

The previous test required rotated runs to exhibit a known autopilot difference.
After the force-path correction the entire rotated run passes the existing
tolerances; the test now requires full success, with no tolerance relaxation.

## Artifacts

The raw game copy is `artifacts/game-suite-v2-2026-10-19.txt`, SHA256
`F6871A1FA68DF4DA46882E7C7EB6D225C4990FB01A93222F1FDE6A374CDD3C8F`.
The repaired simulator LCD export is `artifacts/simulator-suite-v2-sleep.txt`;
the full strict comparison is `artifacts/game-comparison-suite-v2-sleep.txt`.
`artifacts/comparison-suite-v2-summary.json` contains paired case measurements.
These artifacts preserve this measurement even if the user updates result.log.
The shared in-game script has not changed, so no script reload is needed for
these simulator-only repairs.

Verification: all 323 tests pass (293 simulator, 30 validation). The native
launcher completes 19 cases / 133 samples. Full game equivalence still fails;
these force-path repairs do not resolve the recorded +Y/coast discrepancy.

## Complete compact suite-v3 game measurement

The subsequent game measurement is complete: 54,542 characters after newline
normalization, 19 cases, 133 samples, and 123 trace rows / 27,125 trace characters
with none dropped. It ends with `END|COMPLETE|19|133`. This confirms the compact
report fits the actual 65,000-character LCD publication limit. The matching
simulator report contains 48,871 characters and the same sample/trace counts.

All six translation cases now show approximately 2.1571 m of displacement and
2.8518 m/s final speed in the game, versus 2.1690 m and 2.8602 m/s in the simulator.
The game coast pulse moves 0.4135 m and ends at 0.4690 m/s, versus 0.4211 m and
0.4767 m/s in the simulator. The earlier +Y/coast missing-motion symptom is absent
with the v3 settling procedure; this observation alone does not prove its cause.

The strict comparison still fails. Nine settling durations differ, producing
cumulative run-time offsets in later samples. Stored battery energy also differs;
comparison must distinguish elapsed settling time and battery update phase from
energy integration errors. Attitude and inherited absolute-position differences
remain and must be evaluated relative to each case's initial state as well.

Two substantial motion discrepancies remain independent of missing LCD data:

- During battery discharge, the game reports 2,880 N but moves only about
  0.0074 m and reports zero final velocity. The simulator travels about 4.783 m
  and reaches 1.9068 m/s. Treat the game's thrust telemetry and actual motion
  separately; the log does not establish why they disagree.
- During waypoint flight, the simulator reports autopilot enabled and thrust
  delivery but no movement. The game travels approximately 27.3 m, ending
  2.7049 m from its waypoint. The game also enables dampeners when autopilot
  starts; the simulator leaves them disabled. The corresponding control and
  rigid-body activation paths require source investigation.

The game measurement is archived at
`artifacts/game-suite-v3-complete-2026-10-19.txt`; the simulator counterpart is
`artifacts/simulator-suite-v3.txt`. Full strict comparison output is
`artifacts/comparison-suite-v3.txt`. No tolerances were relaxed and the failed
comparison is not presented as game equivalence.

## Waypoint activation investigation and repair

The frozen simulator waypoint case is a missing control-state side effect,
not evidence that callbacks or physics steps stopped running. A native small-grid
regression first lets Havok deactivate for 600 frames, enables a OneWay waypoint,
and observes thrust and movement. Before repair it records 10,000 N but exactly
zero displacement. Existing autopilot tests used awake large grids, whose solver
deactivation is disabled, and therefore missed this precondition.

The decompiled activation chain is:

1. `MyAutopilotComponent.SetAutopilot` marks the thrust component dirty, and
   `OnSetAutoPilotEnabled` calls `ResetShipControls` to enable dampeners.
2. Each `MyEntityThrustComponent.AutoPilotControlThrust` write also marks it dirty,
   including zero commands through `AutoPilotControlThrustDampenersEnabled`.
3. `MyThrusterBlockThrustComponent.OnControlTrustChanged` calls `MarkDirty`;
   the base implementation sets `m_thrustsChanged`.
4. `MyEntityThrustComponent.UpdateBeforeSimulation` activates sleeping physics
   when that flag is set, before the active-only thrust application.

The simulator now activates physics on autopilot transitions and command updates,
and restores shared dampeners on enabling autopilot. It keeps the existing
active-only force semantics: simply delivering thrust must not wake every body.
Both new regressions failed before repair and pass afterward. No game-loop
reordering or validation-script change was needed.

The fresh LCD export `artifacts/simulator-suite-v3-autopilot-wake.txt` completes
19 cases / 133 samples in 49,075 characters. Waypoint distance falls from 30 m
to 2.353454 m after 600 frames, versus 2.7049 m in the recorded game measurement.
Shared dampeners are now enabled in both hosts. The strict game comparison still
fails; output is `artifacts/comparison-suite-v3-autopilot-wake.txt`.

Verification: 296 simulator tests pass; 36 of 37 validation tests pass.
`RotatedRun_NormalizesTheEntireSuite` now fails on waypoint thrust/power samples
and one velocity sample. Before repair, both compared waypoint runs stayed asleep,
masking this approximate-controller discrepancy. The test remains strict and is
not weakened to obtain a green result. Source inspection and independent review
find no coordinate transformation introduced by the activation repair, but the
rotation discrepancy remains unresolved. Battery-discharge sleep behavior and
the previously recorded settling/energy differences also remain open. The existing
complete game log is sufficient for the next investigation; no rerun is requested.

## Waypoint control and braking follow-up

Further source-backed repairs address the remaining trajectory error:

- Target displacement now uses the remote's `ReferencePosition`, not the grid
  origin. An offset-remote regression previously produced spurious lateral thrust.
- A zero forward-drive command still passes through `ComputeAiThrust`, applying
  velocity dampening. The previous early return skipped this braking path.
- The remote speed setting gates acceleration; it is not a physical velocity
  clamp. The simulator's post-physics clamp was removed. A ship already moving
  at 10 m/s with a 3 m/s remote limit now decelerates by force*dt/mass rather than
  snapping to 3 m/s. The prior top-speed test incorrectly assumed an instantaneous
  cap; it now checks the analytic one-frame overshoot bound instead. Strict game
  comparison tolerances were not changed.
- AI thrust ratios replace infinite quotients with 1, just like the game's
  invalid-number guard. Finite ratios remain unchanged.
- Autopilot transform/projection operations now preserve the source's precision:
  inverse double world matrix before narrowing, double target/drift transforms
  before subtraction/casting, and float velocity rejection before widening.
- Final OneWay arrival uses the destination trigger radius (small grid: gridSize;
  large grid: gridSize*0.25), rather than the intermediate gridSize*3 radius.
  Arrival disables the remote autopilot while retaining its waypoints. The
  default-radius tests now place intermediate targets relative to the remote.

Focused behavioral regressions failed before their corresponding repairs.
Verification now reports 305 simulator tests passing and 36/37 validation tests
passing. `RotatedRun_NormalizesTheEntireSuite` remains the only failure, on
waypoint velocity/thrust samples; it is kept strict. Full game equivalence still
fails and the approximation is not declared fixed.

The native waypoint endpoint is now 2.616685 m from the target, versus 2.7049 m
in the v3 game measurement: approximately 8.8 cm difference, within the distance
tolerance. Endpoint speed still differs (about 2.224 versus 2.348 m/s), so distance
alone is insufficient. An isolated native replay seeded from the game's recorded
starting attitude also shows the residual difference; it cannot be explained
solely by inherited attitude drift from preceding cases. That replay is an
analysis probe, not a second implementation of the validation script.

To inspect initial command timing and braking without guessing a delay or tuning
constants, suite v4 adds 88 waypoint observations to the unchanged canonical
19 cases / 133 samples. The LCD includes tick zero, every frame through tick 60,
then 20-frame checkpoints through tick 600. V3 only contains seven waypoint
samples, which cannot resolve the first-frame behavior. A fresh v4 game report
is now requested. The blueprint and test maneuvers remain unchanged.

The new native LCD export is `artifacts/simulator-suite-v4.txt` (55,684 characters).
It passes its strict schema check and includes all transition/waypoint diagnostics.
Native test output is `artifacts/autopilot-final-tests.log`; the earlier v3
counterparts remain archived. The new packed script compiles with PB wrapper
imports. No timing delay or force-smoothing constant was invented to match the
measured endpoint.
