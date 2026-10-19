# Single-paste station validation

Paste **FleetCommand_Validation_Station_v1**. It contains a static large-grid
station and a small-grid validation craft already connected. The craft is the
earlier fixture with a full-size small-grid connector on a nose extension, plus
a second LCD. Tiny ConnectorSmall ports cannot mate with large-grid connectors.
Both grids have damage disabled to exclude exhaust/contact damage from this
physics experiment. The station is a 22.5 m square backing wall with a powered
port. Keep at least 100 m of clear space around the flight path.

Load the local **FleetCommand.Validation** script into the craft's **FC Validation**
programmable block. Use survival physics, no gravity/atmosphere, and leave the
cockpit and remote control. Run **`dockprobe`**. No exact world position or attitude
is required. The mission captures the initial mating transform and reference frame.

The mission:

1. Records a connected baseline, releases both ports and disables attraction.
2. Retreats 40 m from the station and settles at a measured standoff.
3. Runs the six-case `flightprobe`: full versus limited power from rest and a
   measured moving start, then a forward waypoint and a turn/arrival/braking case.
4. Returns to the recorded mating position and attitude using PB velocity/gyro
   commands, then enables the ports for a final approach limited to 0.5 m/s.
5. Observes **60 consecutive frames** of Connectable without locking, issues an
   explicit Connect command, then records a 120-frame connected hold.

Successful native execution takes approximately three minutes and ends docked.
The assembly paste and full mission have also completed in the game; the retained
reports and comparison are below. If the pasted craft
is not initially Connected, the script reports `START_CONNECTED` and stops rather
than inventing a starting state. Preserve that error and inspect the paste.

Two LCDs are the complete artifacts:

| LCD | Save complete text to | Expected first line / final marker |
| --- | --- | --- |
| **FC Dock Report** | `TestResults/result.log` | `FCDIAGNOSTIC\|1\|station-v1\|dock-v1`, `END\|COMPLETE\|9\|…` |
| **FC Report** | `TestResults/flight.log` | `FCDIAGNOSTIC\|1\|craft-v1\|flight-v1`, `END\|COMPLETE\|6\|…` |

Copy through **Edit Text**, including the header, columns, all samples and footer.
Do not concatenate or truncate the reports. Docking samples include phase and
tick, relative COM motion/attitude, error from the captured mating position,
both connector states and six thrust outputs. The flight report contains raw
power, thrust and motion channels; see [flight-probe.md](flight-probe.md).
Each report is bounded below 64,000 characters. The final native exports are
47,426 and 50,605 characters respectively. `abort` clears thrust/gyro/autopilot
commands and enables dampeners. Reload never resumes an interrupted maneuver;
it stops actuation and marks the saved report interrupted. After an interruption,
restore a connected fixture before starting again.

The independent `flightprobe` command remains available on an undocked craft.
Existing `start` and `restprobe` commands retain their separate experiments.
The old canonical `compare` command does not parse these diagnostic schemas;
these raw artifacts are retained for source investigation and game comparison.

## Native execution

```powershell
dotnet test FleetCommand.sln --no-restore -p:MdkBuildConfiguration=PackExplicitly -m:1
./FleetCommand.Validation.Launcher/bin/x64/Debug/net48/FleetCommand.Validation.Launcher.exe dockprobe artifacts/station-docking-simulator.txt
./FleetCommand.Validation.Launcher/bin/x64/Debug/net48/FleetCommand.Validation.Launcher.exe flightprobe artifacts/flightprobe-simulator.txt
```

The mission launcher exports the actual **FC Dock Report** as the requested file
and actual **FC Report** as the sibling `.flight.txt` file. It runs the same shared
experiment host and MDK runner as the PB; no duplicate maneuver implementation.
Use `--frame-from game-docking-report.txt` to replay the initial reference attitude.
The loader preserves serialized grid placement/static flags and restores the
initial paired connector IDs. It does not teleport during the mission.

Regenerate the checked-in assembly with `tools/New-StationValidationBlueprint.ps1`.
It copies the original single-grid fixture and applies the documented station
changes. `tools/Deploy-Validation.ps1` copies the named assembly and script to
AppData with hash verification and backs up changed files.

## Simulator corrections and limits

The connector source audit corrected four assumptions before using this fixture:
capture does not automatically lock; imported mating anchors and detector radii
come from model dummies; full-size small-grid ports can mate with large-grid
ports; and connector maintenance/automatic detection use the source 10/40-tick
cadence. Native fixed/malleable constraints provide the physical interaction.

The simulator supports the physical two-grid connection. Cross-grid power
distribution, terminal discovery and full construct semantics remain incomplete;
this mission does not silently claim to validate them. The craft has its own
reactors/battery and the station has its own battery. Waypoint completion is
observed in the report, not assumed to be equivalent. Current tests cover two
world orientations, initial pairing, missing connection, abort, reload during
retreat, and loss/reacquisition of capture. All 349 simulator and 64 validation
tests passed in `artifacts/station-validation-full-tests.log`.

Opening an LCD's text editor blocks `WriteText` in the game. The experiment now
keeps collecting while publication is blocked, then flushes the buffered report
after the editor closes. On completion, actuators stop even if publication must
wait; Update10 retries remain scheduled until both reports are published.
`status` displays the current phase without requiring an LCD editor. The native
model covers both WriteText overloads. All 350 simulator and 65 validation tests
pass in `artifacts/lcd-editor-full-tests.log`, including a complete mission with
both editors held open until after completion. The earlier stopped game run has
no terminal status captured, so its cause remains unconfirmed.

## Completed game measurement, 2026-10-19

The complete reports are archived as
`artifacts/game-station-dock-v1-2026-10-19.txt` and
`artifacts/game-station-flight-v1-2026-10-19.txt`. Neither overflowed. Docking ends
with `END|COMPLETE|9|262`; the flight report ends with `END|COMPLETE|6|4958`.
The native replay at the recorded initial docking attitude is
`artifacts/station-docking-game-frame-2026-10-19.txt` and its `.flight.txt` sibling.
Both native reports also complete. These diagnostic completion markers confirm
execution, rather than asserting numerical equivalence.

Original game reports are also checked in under
[`validation-data/2026-10-19/`](../validation-data/2026-10-19/), with hashes and
replay instructions in [validation-data/README.md](../validation-data/README.md).
Paths under `artifacts/` refer to ignored local investigation outputs.

| Measurement | Game | Native replay |
| --- | ---: | ---: |
| Full-power rest, 180-frame velocity gain (m/s) | 5.244481 | 5.244418 |
| Full-power moving, velocity gain (m/s) | 5.252841 | 5.252675 |
| Limited-power rest, velocity gain (m/s) | 0.963996 | 0.009101 |
| Limited-power moving, velocity gain (m/s) | 1.034307 | 1.010723 |
| Forward waypoint final distance (m) | 0.075717 | 0.080474 |
| Turning waypoint final distance (m) | 0.077021 | 0.086490 |
| Final locked mating position error (m) | 0.000029 | 0.017056 |
| Mission duration (ticks) | 10486 | 10248 |

Both waypoint cases finish stationary with autopilot off. Both docking runs
observe capture before explicit Connect, followed by 120 locked-hold frames.
The large limited-power-from-rest discrepancy remains open; matching power
telemetry alone does not validate the low-speed physics. Limited-power moving
gain differs by about 2.3%, and final connector seating also requires review.
The replay restores starting world attitude, not the complete native constraint
history or the attitude at the later flight start. No force thresholds, tolerances,
or native physics parameters were fitted to these measurements.

Game flight mass is 8221.930664 kg versus fixture mass 8222 kg. Its initially
connected docking metadata reports 8321.962891 kg; that separate 100 kg offset
must be explained before using connected mass metadata as a conformance claim.

### Power discrepancy resolved: thrust precedes distribution

The table above preserves the original comparison before the correction.
Source `MyThrusterBlockThrustComponent.Schedule` runs thrust at priority 6;
`MyGridResourceDistributorSystem.Schedule` runs distribution at priority 14.
`UpdatePowerAndThrustStrength` publishes the new required input, but
`Sink_RequiredInputChanged` only marks distribution dirty. `UpdateThrustStrength`
and `ApplyThrustModifiers` still read the existing cached supplied ratio.
The simulator incorrectly distributed the new demand before aggregating thrust.

The world now aggregates thrust before distribution. This restores the observed
single full-power frame when five thrusters start on the battery-only supply.
That initial impulse changes the subsequent native low-speed constraint behavior;
the previous near-stall was not evidence of incorrect steady-state battery power.
Grid initialization establishes supply ratios without transferring energy twice,
matching the initial distribution in `MyCubeGridSystems.OnAddedToGroup`.

| 180-frame velocity gain (m/s) | Game | Corrected native replay |
| --- | ---: | ---: |
| Limited-power rest | 0.963996 | 0.963972 |
| Limited-power moving | 1.034307 | 1.034302 |

Corrected full-power gains remain 5.244418 and 5.252674 m/s. Waypoint endpoints
remain stationary, at 0.080193 and 0.086446 m. The mission completes and stays
connected; connector seating still differs (native 0.016954 m mating error).
No physics constants, limits, validation tolerances, or game-script commands
changed. No additional game measurement is required for this correction.

Artifacts: `artifacts/station-docking-power-order-2026-10-19.txt` and its
`.flight.txt` sibling. Regression failure is recorded in
`artifacts/power-demand-order-red.log`; the separate startup regression failed
in `artifacts/power-startup-red.log` before initialization was corrected.
All 351 simulator and 66 validation tests pass in
`artifacts/power-demand-order-final-tests.log`.
