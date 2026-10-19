# Survival measurement conformance repairs

Input: `TestResults/result.log`, recorded in Survival with no other objects.
The original log is preserved. It completed all 19 cases and 133 samples;
completion means measurement collection succeeded, not simulator equivalence.

## Established corrections

| Area | Evidence and correction |
| --- | --- |
| Thrust application | `MyThrusterBlockThrustComponent.UpdateThrusts` applies the resultant at physical-group COM. Replace individual mount-point impulses with a single linear impulse; off-centre mounts no longer introduce false torque. |
| Dampeners | `MyEntityThrustComponent` enables dampening independently of pilot occupancy. Unoccupied working controllers retain grid dampening; stale unoccupied MoveIndicator is ignored. `ReduceSlowdownByOverride` prevents opposing overrides being cancelled. Overridden/disabled thrusters are excluded from controllable capacity. |
| Discovery | Game reports 17 terminal blocks; simulator returned all 183 physical blocks. Terminal queries exclude structural Standard blocks while retaining them for geometry/mass and framework mutation. |
| Mass | Installed `Components.sbc` and cube definitions give fractional masses: PB138.4, remote38.2, small gyro613.6, battery146.4, LCD89.8, connector204.8 kg. Truncation lost 3.8 kg across this fixture. Restored supported definition values, giving 7551.8 kg at spawn vs game7551.786621 after fuel use. |
| Thruster demand | `UpdatePowerAndThrustStrength` scales maximum consumption by command and applies one idle floor to the electrical group, excluding overridden thrusters from that floor. Previously all six thrusters requested maximum while idle. Imported utility, gyro and thrust sink priorities/adaptability now follow `ResourceDistributionGroups.sbc`. Controller/autopilot demand is prepared before power allocation. |
| Charge accounting | `RecomputeResourceDistributionPartial` accounts for storage sinks in producer output. Charge energy now appears in source draw/fuel consumption. Recharge storage takes priority; Auto storage cannot charge from its own unused output. Source tiers share draw by capacity. |
| Idle loads | Add actual PB0.0005, cockpit0.003, LCD0.00004 MW plus remote idle0.0000001 and ion idle0.000002 MW each. Along with two gyros this reproduces measured battery output0.007552 MW, battery input0.2 MW and reactor output0.2 MW. |
| Battery energy | `MyBatteryBlock.UpdateAfterSimulation100/UpdateInternal/StorePower` samples current input/output every100 frames with definition recharge multiplier0.8. Imported batteries use that cadence; synthetic equation fixtures retain explicit per-step behavior. Absolute update phase in a running game remains unknown. |
| Runtime | `MyProgrammableBlock.RuntimeInfo.TimeSinceLastRun` constructs a TimeSpan with166666 ticks per frame. Remove .NET Framework millisecond rounding from simulator runtime. |
| Waypoint setup | `CanEnableAutoPilot` rejects one waypoint except OneWay mode. SimRemoteControl now stores FlightMode and enforces activation gating. Shared script explicitly chooses OneWay; suite revision advances to2. Archived v1 reports remain parseable, but cross-revision equivalence comparisons fail. |

## Measurement status and next run

Force, mass, discovery, idle power and runtime regressions pass. The repaired
simulation completes the shared suite and exports the complete LCD report.
Its comparison against the archived game run still fails; tolerances are
unchanged. Pre-waypoint observations can diagnose the original defects, but
the old waypoint measurement never enabled autopilot and cannot validate the
new OneWay test.

Remaining observations need a fresh suite-v2 game report: several original
override pulses reported force with almost no translation, and game settling
was often shorter. No contact, sleep or power explanation has been established
for those pulses. The user offered to rerun after patching and mentioned a
possible accidental bump. Neither explanation is assumed in simulator code.

The simulator autopilot still uses an approximate steering/braking model and
shows world-rotation sensitivity. Battery sample comparisons can also differ
because game100-frame scheduling is not synchronized to PB start. Dynamic
fuel mass tracking, full collision behavior and other uncovered features are
outside this repair's equivalence claim. These limits remain visible rather
than being hidden by wider tolerances.

## Reproduce

Verification: 318 tests passed (288 simulator and 30 validation). MDK packing
and exact packed-source compilation passed with zero warnings/errors. The
native launcher exported `artifacts/simulator-suite-v2.txt`: 19 cases,
133 samples, 40189 characters, with a complete footer. The script was copied
to the game's local library; the checked-in blueprint files did not change.
Independent review identified the group idle-floor issue above, now covered
by four regression cases. The original game log remains unchanged (SHA256
`C1A786C6A8363627A2D811CDF323A725931A29EE311AAC78AFE43458E1E2BF15`).

Run the solution tests with `-p:MdkBuildConfiguration=PackExplicitly` to avoid
the existing main project's automatic deployment. Build the validation
project normally to pack its script, then use `tools/Test-PackedValidation.ps1`.
Run the launcher with `run artifacts/simulator-suite-v2.txt`; it extracts the
report from the simulated LCD. Use `compare` on simulator and game reports
from the same suite revision. `tools/Deploy-Validation.ps1` installs the
checked-in fixture and packed local library script; already placed PBs need
the updated library script loaded explicitly.
