# Suite v2 game comparison

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
