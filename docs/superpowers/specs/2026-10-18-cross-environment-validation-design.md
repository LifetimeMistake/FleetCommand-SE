# Game and simulator validation

## Purpose and evidence

Run the same programmable-block test logic against the same vessel in Space
Engineers and FleetCommand.Test. Save observations that expose behavioral
differences rather than merely repeating the simulator's own expectations.

This continues the intent in
`docs/superpowers/plans/2026-09-24-fleetcommand-cross-env-equivalence-test.md`
and the latest Claude Code session. The current request replaces the earlier
manual craft-building workflow with a generated SBC blueprint and explicit game
deployment. Automated tests must never access AppData. Local decompiled game
code and installed block definitions remain the source for implementation facts.

## Selected approach

Use a separate MDK project with a thin Program bootstrapper and an API-only
validation runner. The simulator invokes that exact runner with its ingame API
implementations through its existing fixed-step callback mechanism. Program
only supplies services, forwards Main/Save, and transfers Storage. No host flag
may change test commands, calculations, or case order.

Alternatives considered: two independent scripts could diverge and weaken the
comparison; booting the complete game scripting engine in the simulator adds
scope without first improving the behavioral measurements. The shared runner
validates API and physics behavior; it does not establish equivalence of the
game's script compilation, instruction accounting, or sandbox execution engine.

## Vessel and fixtures

Create a base-game, single-grid, small-grid craft with six ion thrust directions,
gyros in more than one orientation, remote control, cockpit, programmable block,
reactor, battery, connector, and report LCD. Use armor mounting structure and
verify exact sizes, orientations, mount areas, mass, power, and thrust against
installed definitions. Keep thruster exhaust clear of the craft. Select final
actuator counts from the resulting mass and power budget.

Store canonical bp.sbc fixtures in the repository. Validate through the existing
SBC reader and template converter, then spawn and execute in the simulator.
Simulator acceptance alone cannot certify in-game mount legality; game loading
and physical operation remain a separate acceptance step.

Preserve block identity and required operating state across SBC conversion:
custom names, enabled state, battery charge/mode, and relevant controller and
actuator settings. Do not silently replace fixture state with convenient simulator
defaults. Keep parser structural validation separate from simulator adaptation.

Provide a separate docking-target fixture for two-grid connector cases. Both
fixtures and the script are copied to game directories only by an explicit
deployment step, never during tests or ordinary builds. Replace the five existing
AppData-dependent tests with repository fixture tests as part of this work.

## Execution and measurements

Run first in empty space, with no natural gravity, atmosphere, mods affecting
physics, pilot input, or nearby obstructions. Record preflight observations and
reject unsuitable conditions before moving. Record test revision, fixture
revision, case identifiers, mass, available thrust, battery state, and cadence.
Record the game version and world settings in the host-side comparison manifest.

Use explicit start, status, abort, and export commands. Cache blocks once.
Execute bounded state-machine steps on Update1; detect unexpected callback gaps
and record them. Each case has a bounded settling phase and timeout. Abort and
completion clear commanded overrides and disable autopilot. Power experiments
must leave a supply available for the PB and report LCD.

Record displacement and velocity in the initial reference frame, and attitude
relative to the initial orientation. Retain elapsed simulation time with each
sample. Record only observable ingame API quantities; inferred acceleration or
torque is labeled derived, never presented as directly measured gyro torque.

Cover block discovery and API state, LCD text, six-axis thrust and coast,
gyro channel signs and rotated mounts, override response and angular decay,
dampener braking, reactor/battery supply and charging, power limitation,
straight waypoint flight, and connector connect/disconnect with a target.
Exercise dampener braking after a thrust pulse: a PB cannot inject physical
cockpit input through the ingame API. Manual pilot-input behavior must be marked
as requiring a separate interactive test. Fuel depletion is included only when
the existing simulator can represent the fixture inventory and consumption;
otherwise report it explicitly as uncovered. Natural gravity requires a separate
matched planetary scenario. Coverage must distinguish tested, manual, and
unsupported behavior rather than claim universal equivalence.

## Reports and comparison

Write bounded, versioned, invariant-culture records to the LCD and PB Storage.
Preserve the last complete report on reload; do not automatically resume a
physical maneuver after loading. Include a completion marker, sample count,
and truncation/error status so partial output cannot pass comparison. Keep logs
below the modeled 100,000-character LCD limit; use bounded per-case sampling.

The simulator exports the same report format to test artifacts. A host-side
comparison tool accepts two explicitly supplied report files and checks schema,
case coverage, fixture/preflight compatibility, and timing before comparing
measurements. Use exact comparison for discrete API behavior and explicit
per-channel absolute/relative tolerances for dynamics. Choose provisional limits
from precision and sampling uncertainty, label them provisional, and review them
against the first real game run. Never widen limits automatically to hide a
mismatch. Report missing cases and incompatible initial conditions separately
from physical divergence.

## Acceptance and delivery boundary

Build the MDK bundle with sandbox analyzers enabled. Run the repository suite
with no AppData reads; prove fixture conversion/spawn, runner completion and
abort cleanup, report persistence, and comparison rejection of incomplete or
altered reports. Save a simulator baseline and deploy the generated blueprint
and script to the user's game folders when filesystem permissions allow.

The user loads the craft in game and runs the script, then supplies its exported
report for comparison unless a supported local game-control mechanism is found.
Until a real game report has been captured and compared, delivery establishes
the validation workflow and simulator baseline, not measured game equivalence.

## Review status

Approved by the user on 2026-10-18 with these requirements: keep the expandable
script project as the single source of running test logic; add a separate console
launcher that loads the checked-in vessel, runs that logic in the simulator, and
reads the LCD from outside the simulation; place the entire report and every
comparison datum on the LCD. Storage is backup only. Normalize translation,
orientation, and vector measurements to the initial controller frame. Execute
the implementation plan natively without another approval handoff.
