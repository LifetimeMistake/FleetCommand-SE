# Game / simulator validation

`FleetCommand.Validation` is the single source of test code. Its MDK `Program`
only forwards ingame services to `ValidationRunner`. The console launcher runs
that same compiled runner against FleetCommand.Test, then exports **the text read
from the simulated LCD**. It does not construct a second telemetry report.

## Build, run, compare

Windows, .NET Framework 4.8, x64, the installed game, and the existing
gitignored `MDK.paths.props` are required. The script pins Roslyn 4.12 because the
installed MDK analyzer needs that compiler version; the script still uses C# 6.

```powershell
dotnet build FleetCommand.Validation.Launcher/FleetCommand.Validation.Launcher.csproj
./FleetCommand.Validation.Launcher/bin/Debug/net48/FleetCommand.Validation.Launcher.exe run artifacts/simulator-report.txt
./FleetCommand.Validation.Launcher/bin/Debug/net48/FleetCommand.Validation.Launcher.exe compare artifacts/simulator-report.txt artifacts/game-report.txt
dotnet test FleetCommand.sln -p:MdkBuildConfiguration=PackExplicitly
```

`run [report-path] [blueprint-path]` accepts an explicit fixture path. The default
fixture is copied from the repository to the launcher output at build time.
The default spawn deliberately has a nonzero position and rotated attitude.
Exit codes: 0 = completed run / comparison within provisional limits;
1 = incomplete run / incompatible reports / measured differences;
2 = usage or host error. A COMPLETE run means data collection finished, not
that the simulator has been proven equivalent to the game.

Build output stays in `artifacts/validation-script/FleetCommand.Validation/`.
Validation builds keep their output local. The test command disables automatic
MDK packaging across the solution; test code never reads game AppData. Deployment is explicit:

```powershell
./tools/Deploy-Validation.ps1
```

This copies the two named blueprints and the packed script into the standard
SpaceEngineers AppData directories. It verifies hashes, backs up changed files
as `.previous`, and removes only the named generated blueprint's binary cache.
It does not change your other craft. SE can refresh blueprints without restarting.

## Game procedure

1. Use an unmodded empty-space world with scripts enabled. Use survival physics,
   realistic inventory multiplier, no gravity/atmosphere, and no nearby objects.
   Creative tools may be used to paste the blueprint; record the actual game
   version, world settings, mods and inventory multiplier with the extracted report.
2. Paste `FleetCommand_Validation_v1`. It is a 183-block small grid: armor deck,
   remote, cockpit, PB, six ion thrusters, two differently mounted gyros, three
   fueled reactors (1.5 MW total), one 0.2 MW battery and one LCD/connector.
   Default battery charge is 0.025 MWh. Keep the dock away during the main run.
3. Load `FleetCommand.Validation` into `FC Validation`. Exit the cockpit/remote,
   leave the craft at rest, and run the PB with argument `start`.
   Position and orientation are arbitrary. Start battery charge must be between
   0.02 and 0.04 MWh; paste a fresh fixture if it has charged outside that range.
4. Leave the craft untouched until the PB says COMPLETE (roughly 1–3 minutes).
   `status` prints progress, `abort` clears commands, and `export` rewrites the
   entire retained report onto `FC Report`. A callback gap above 25 ms aborts
   with a cadence error; retry in a world that sustains the required tick cadence.
5. Open **FC Report's text editor**, select all and copy the full text, including
   the `FCVALIDATION` header, all `S` rows and final `END|COMPLETE|19|133` line.
   Save that text as `artifacts/game-report.txt`. A screenshot or visible LCD
   viewport is insufficient: it cannot expose the complete data. Close the editor
   while a run is active, because it may prevent script writes.
6. Run the comparison command. Send both text files and the world settings for
   investigation. Failed/aborted/truncated runs cannot pass comparison.

The LCD contains the complete report (the current simulator baseline is 40,418
characters), not a summary.
PB Storage is backup. Saving/reloading never resumes a physical maneuver:
completed reports remain available; a saved partial run is marked INTERRUPTED.
Ingame instruction accounting and the actual game script engine still require
the first live game execution even though MDK analysis and packaging pass.

## Measurements and cases

At `start`, the runner captures the remote's orientation and center of mass.
Every position is displacement from that initial COM, expressed in that fixed
initial controller frame. Linear/angular velocities use the same initial axes.
Attitude is the normalized relative quaternion, not world Euler angles. The
frame stays fixed while the ship rotates. Battery readings are **changes from
the starting charge**, whose absolute value is also saved in the metadata.

Each case records seven samples: its initial observation plus six equally spaced
samples over a fixed number of ticks. A row records case/tick, run time, observed
callback dt, relative position/velocity/angular velocity/quaternion, each of the
six actual thruster outputs, battery input/output/charge change, aggregate reactor
output, dampener/autopilot flags, connector status and distance to the waypoint.
Tick-zero actuator telemetry describes the preceding completed physics interval;
new commands take effect on the following interval in both hosts.

The 19 cases cover baseline/LCD state, six thrust directions, coast, three gyro
axes, a rotated gyro mount, angular decay, dampener braking, battery discharge
and recharge, constrained power, a relative forward waypoint and unpaired
connector calls. Between cases an API-only velocity/rate controller settles the
craft; its duration is recorded and bounded. Abort/completion clear thrust/gyro
overrides, stop autopilot, restore reactors/battery auto and enable dampeners.

This is an expandable validation suite, not a claim that every simulator feature
is exercised. `COVERAGE` lines enumerate automated, manual and uncovered behavior.
For manual connector checks, paste `FleetCommand_Validation_Dock_v1`, align the
connectors until Connectable and run `dock`; after it becomes Connected, run
`dock` again to log the state and disconnect. Those observations append as MANUAL
records to the LCD and are deliberately excluded from the automated verdict.
Pilot input, save/reload and paired docking dynamics remain operator-assisted.
Planetary gravity, atmosphere, collisions, fuel depletion, speed caps, IGC,
instruction limits and other block variants are not covered by this first suite.

## Comparison rules

Comparison requires the exact schema/fixture/suite revisions, all 19 cases,
their seven samples, valid timing/quaternions/finite numbers and the completion
footer. Block capability/coverage mismatches fail. Mass must agree within
0.05 kg; grid size is exact. Initial charge may differ within preflight bounds,
since charge deltas are compared. Terminal-system enumeration differences are
reported as API differences while other channels are still compared.

These limits are **provisional**, chosen before a live game measurement. No limit
is widened automatically:

| Channel | Absolute limit | Relative limit |
| --- | ---: | ---: |
| Position | 0.05 m | 1% |
| Linear velocity | 0.03 m/s | 1% |
| Angular velocity | 0.005 rad/s | 1% |
| Attitude | 0.01 rad quaternion angular distance | — |
| Thruster output | 0.5 N | 1% |
| Power | 0.001 MW | 1% |
| Battery charge change | 0.00001 MWh | 0.1% |
| Waypoint distance | 0.1 m | 1% |
| Run time / callback dt | 0.05 s / 0.001 s | — |
| Flags / status | exact | — |
| Settling duration | 1 tick | — |

Numeric limits add their absolute and relative terms. There is no bitwise
floating-point equality claim. Quaternion sign is handled by angular distance.

## Findings already exposed

- Rotated fixtures exposed the simulator composing block matrices as
  `grid * local`. The game uses `local * parent` (MyPositionComponentBase.cs
  lines 457/567). This is fixed with a regression test.
- SBC conversion previously dropped names, startup state and reactor fuel, and
  computed rotated block centers from unrotated sizes. The fixture path now
  preserves serialized fields and applies the startup state used by this suite.
  Reactor uranium consumption is mapped using MaxPowerOutput / 3600 / 1 kg,
  following MyReactorDefinition.FuelInfo and the default fuel multiplier.
- Whole-suite translation invariance passes. Rotated runs agree through the
  pre-waypoint flight probes within the stated limits, but the simulator's
  approximate autopilot amplifies small differences. The comparator flags those
  waypoint differences and their subsequent attitude effects; a regression test
  preserves that finding. This remaining simulator issue is not hidden by
  looser limits or described as a passing game-equivalence check.
- The simulator currently enumerates structural armor as terminal blocks; the
  real game's terminal system may enumerate fewer blocks. The raw count is
  retained so comparison can expose this representation/API discrepancy.
- LCD writes now publish on the simulator's ten-tick update, following
  MyTextPanel.UpdateAfterSimulation10/UpdateModApiText. The script waits for its
  append/read probe to become visible before replacing it with report text;
  the launcher also lets the final write publish before extracting the LCD.

Minor hardening deferred from final review: the comparator counts capability
records without enforcing unique thruster indices, and checks monotonic sample
times without reconstructing the complete timing schedule. Normal runner output
is covered; identically malformed hand-edited reports can evade those two checks.

## Extending

Add cases in `ValidationCase.CreateSuite`, implement their commands in the shared
runner, and add any new ingame observations to the LCD schema. Update the fixture
and revision strings together. Update the strict host parser/sample contract and
regression tests when the schema changes. No simulator-specific test logic goes
in the MDK project. The generator `tools/New-ValidationBlueprint.ps1` reproducibly
rebuilds the checked-in SBCs; verify changed fixtures in the parser and simulator
before deploying and checking their legality in game.
