# Low-speed isolation diagnostic

Reload the local **FleetCommand.Validation** script into the existing test vessel's
programmable block, leave both cockpit and remote control, and run **`restprobe`**.
Use the same zero-gravity, disconnected fixture as the ordinary suite. A fully
charged battery is accepted; an empty battery is rejected. This command replaces
the LCD report, so retain any earlier measurement you need first.

The craft settles before each case. Leave it untouched until completion. The
complete report is written to **FC Report**, beginning with
`FCDIAGNOSTIC|1|craft-v1|low-speed-v5` and ending with
`END|COMPLETE|6|108`. Copy all LCD text to `TestResults/result.log`.
The report remains below the 65,000-character LCD cap in native verification.

Version 5 keeps the version 4 maneuvers and sampling, adding mandatory
`META|initial_qx/qy/qz/qw` fields for the reference controller's world orientation.
Motion data remain relative to the initial frame. This distinction matters:
Havok's constrained low-speed solver produces different gyro/thrust responses at
different world attitudes. Do not align the craft manually to an exact origin
or attitude; retain these fields so the launcher can replay its actual attitude.

| Case | Intervention |
| --- | --- |
| `low_hold` | Five seconds at 20% +X override from rest, with reactors off and battery in Discharge. The override is set once. |
| `low_wake` | Same thrust and power, plus a dampeners-on/off pair in every PB callback. Both writes complete before the next physics interval, leaving dampeners off. |
| `low_refresh` | Same thrust and power, writing zero then 20% override in every PB callback to dirty and reschedule thrust. |
| `low_moving` | Full +X thrust with reactors available until measured velocity along thrust reaches 1 m/s, then five seconds at 20% with reactors off and battery in Discharge. Warmup toggles dampeners on/off and refreshes full override every callback; these interventions stop before measurement. |
| `low_gyro_zero` | Same battery-only 20% from rest, plus one powered gyro with override enabled and a zero target. The source permits automatic body deactivation in this condition. |
| `low_gyro_spin` | Same battery-only 20% from rest, plus one powered gyro commanded to 0.01 rad/s about the thrust axis. The source disables automatic body deactivation while the nonzero target is powered. Spinning about thrust retains its world direction. |

All measurements use the run's captured initial reference frame. Samples are
dense for the first 12 ticks, followed by one measurement every 60 ticks.
Each case includes sample zero and the final tick 300 (18 samples per case).
Position, velocity,
attitude, all six thruster forces, battery input/output/energy, reactor output and
dampener/autopilot states are included. Sample zero records commands having been
written, but actuator telemetry still describes the preceding interval. Every
case measures 300 physics intervals. The moving case's sample zero follows
successful warmup and the switch to 20%; intervals 1 through 300 measure reduced
thrust. `U` rows record warmup velocity, force, reactor output and dampeners;
`WARMUPEND` records the measured entry speed. Failure to reach 1 m/s within
600 warmup frames ends with `END|ERROR|WARMUP_TIMEOUT_LOW_MOVING`, without
presenting a stationary run as a moving-start result.

`GY` rows accompany ticks 0, 10 and 300 in the two gyro controls and include enabled,
working and override flags, all three commanded rates, power setting and the
actual thrust axis in the initial reference frame. Angular velocity and attitude
remain in the ordinary `S` rows. Only case-entry writes set these gyro targets:
there is no repeated override refresh or explicit wake during measurement.
The game PB API still cannot expose native `EnableDeactivation` or `IsActive`;
the intended deactivation distinction comes from
`MyGridGyroSystem.UpdateAutomaticDeactivation`, not from direct game telemetry.
In the native simulator regression, the actual Havok flag is checked for both
controls on a rotated craft. A different game result distinguishes conditions;
it does not alone identify the native braking implementation.

The archived v1 game run reached only 0.183 m/s during its fixed-duration
warmup, then stopped before switching to 20%. Its `low_moving` case is not a
valid moving-start control. Version 2 supersedes that protocol.

`low_wake` exercises `MyCubeGrid.EnableDampingInternal`'s activation side effect.
`low_refresh` exercises `MyThrust` override changes and
`MyThrusterBlockThrustComponent.MarkDirty`/scheduling. The PB API cannot expose
native `IsActive`, so the interventions distinguish mechanisms indirectly; a
different result narrows the cause rather than proving every internal step.
No velocity cutoff or artificial force law is assumed.

Native execution uses the same shared runner:

```powershell
./FleetCommand.Validation.Launcher/bin/x64/Debug/net48/FleetCommand.Validation.Launcher.exe restprobe artifacts/simulator-low-speed-v5.txt --frame-from TestResults/result.log
```

The standard `start` command runs the 19-case suite-v5 protocol. Its strict
comparator deliberately rejects diagnostic reports because they are a different
experiment. The new report is analyzed separately from previously archived
canonical game measurements.
