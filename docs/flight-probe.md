# Flight probe

Run `flightprobe` on the validation programmable block in the supplied fixture, in vacuum and zero gravity, with no pilot controlling the craft. The connector must be unconnected and the surrounding flight volume clear. The probe uses only the named blocks on the programmable block's own grid. It leaves a previously disabled connector disabled. `abort` stops the experiment, clears thrust and gyro overrides, disables autopilot, and publishes its footer.

The complete artifact is the `FC Report` LCD, beginning `FCDIAGNOSTIC|1|craft-v1|flight-v1`. Export the entire LCD text, including `COLUMNS`, `CASE`, `GATE`, `ENDCASE`, and final `END` records. Reports stay below the shared 64,000 character limit; overflow is an explicit error. A completed run means that all bounded observation windows ran, and does not assert that the game reached either waypoint.

The six cases are:

| Case | Commands and observation window |
| --- | --- |
| `power_full_rest` | Net +X thrust, with balanced orthogonal load pairs; battery Auto and three reactors enabled, 180 frames |
| `power_limited_rest` | Same thrust/load configuration, battery Discharge and reactors disabled, 180 frames |
| `power_full_moving` | Full-power warmup until measured speed along thrust is at least 1 m/s, then full-power observation for 180 frames |
| `power_limited_moving` | Same measured moving start, then battery-only observation for 180 frames |
| `waypoint_forward` | A target 30 m forward of the case's current reference position, OneWay, speed limit 3 m/s, 1,500 frames |
| `waypoint_turn` | A target 30 m along normalized Right + Backward from the case's current reference position, requiring a turn, same mode/speed/window |

Power cases have dampeners off and zero gyro override. The +X thruster is at full override and -X is off. The +Y/-Y and +Z/-Z pairs are also at full override: their transverse forces cancel while their electrical demand creates a substantial shortage on the 0.2 MW battery. The five jets request about 1 MW; the three reactors provide enough power for the full baseline. This leaves a measurable net +X acceleration, unlike firing all six opposing jets. All six raw forces are exported so pair balance is visible. The moving starts distinguish acceleration from the fixture connector detectors' low-speed effects. Warmup uses only +X and deliberately toggles dampeners on then off and refreshes that override; these interventions stop when measurement starts. A warmup that fails to reach 1 m/s within 600 frames ends with an explicit error.

Between every case the runner applies bounded closed-loop braking, then requires the shared `SettlingGate`'s 15 quiet braking frames and 60 passive frames. Passive rest has zero thrust and disabled gyros. No position or velocity is teleported. A failure to settle within 2,400 frames ends with an explicit error.

`S` rows contain position, velocity, angular velocity and relative attitude in the fixed initial reference frame; the target vector is target minus current reference position in that frame. Initial world quaternion components use round-trip precision in `META` records so a native replay can preserve world orientation. Forces are raw `CurrentThrust` values for all six named directions. Battery input/output/stored energy, each reactor's output, total reactor output, enabled/working masks and autopilot/dampener/connector state are recorded without normalizing power shortages. The first 30 waypoint frames are dense; subsequent samples every 60 frames continue through braking and the arrival tail. Power samples cover the first 12 frames, then every 60 frames.

The turn case moves backward relative to its starting attitude after the forward leg. These two waypoint targets therefore do not deliberately add a second forward leg toward the docking station. Native autopilot collision avoidance remains at the fixture default; collision avoidance, planetary flight and arbitrary obstructions are outside this diagnostic's conformance claim.
