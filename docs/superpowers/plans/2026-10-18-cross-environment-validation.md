# Cross-environment validation implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Deliver a reusable MDK validation script, checked-in craft, simulator launcher, and complete comparable LCD reports.

**Architecture:** FleetCommand.Validation owns all ingame test logic. FleetCommand.Validation.Launcher references the script assembly and existing simulator, runs the shared runner, and exports its LCD. Repository fixtures replace AppData test dependencies.

**Tech Stack:** C# 6 ingame, net48/x64 host, MDK2, existing Havok simulator, NUnit.

**Spec:** docs/superpowers/specs/2026-10-18-cross-environment-validation-design.md

## Global constraints

- Same test commands and calculations in both hosts; no host branches in script.
- All comparison data on LCD, below 100,000 characters, invariant culture.
- All translation, orientation and velocity data relative to initial frame.
- Automated tests never access AppData. Deployment is a separate explicit command.
- Keep Program thin and cache ingame references at construction.

## Review focus

- Arbitrary translated/rotated spawns must preserve normalized observations (task 3).
- Missing blocks, unsuitable initial motion/gravity and cadence gaps must fail clearly (task 3).
- Abort/reload must clear actuators and preserve an extractable report (task 3).
- Truncated, nonfinite, missing-case reports must fail comparison (task 4).
- Unsupported/manual scenarios must remain visible in LCD coverage records (task 3).

## Task 1: Hermetic SBC fixtures and state

Files: FleetCommand.Test/Sbc/{SbcFile,SbcFileReader,SbcFileWriter}.cs,
Templates/{SbcToShipTemplate,SbcBlockAdapters,ShipTemplate}.cs,
Framework/FleetTestContext.cs, Tests/FrameworkTests/Sbc*Tests.cs, blueprints/.

- [x] Add failing tests for named fixture conversion and battery/enabled-state round trip.
- [x] Preserve custom names and needed startup state; correct oriented bounding sizes used by template adapters.
- [x] Generate a base-game craft using confirmed block definitions, check in bp.sbc and a docking target.
- [x] Replace five AppData tests with repository fixture paths and verify parser/converter/spawn tests.

## Task 2: Script project and normalized telemetry

Files: FleetCommand.Validation/{FleetCommand.Validation.csproj,Program.cs,InitialFrame.cs,ValidationRunner.cs,ValidationCase.cs,ValidationReport.cs,mdk.ini}.

Interfaces: InitialFrame converts positions/vectors/orientation axes; ValidationRunner accepts ingame GTS, runtime, PB, echo, prior report; Main(string, UpdateType) drives bounded cases; Report and IsRunning expose state.

- [x] Add failing frame-invariance tests and report contract tests.
- [x] Implement script bootstrap, frame transforms and invariant bounded LCD report.
- [x] Build with MDK analyzers and produce a local packed script without implicit AppData deployment.

## Task 3: Shared behavioral cases and simulator execution

Files: FleetCommand.Validation/ValidationRunner.cs, FleetCommand.Validation.Launcher/{csproj,Program.cs,SimulationRun.cs}, FleetCommand.Test/Tests/Validation/ValidationTests.cs.

Interfaces: SimulationRun.Execute(string blueprint, Vector3D position, MatrixD orientation) returns the LCD text; runner exposes extensible fixed case definitions.

- [x] Add failing end-to-end tests for complete LCD output, translated/rotated spawn, preflight rejection, cadence failure and abort cleanup.
- [x] Implement discovery/API checks, six-axis thrust/coast, gyro axes/mounts/decay, dampener pulse/braking, battery charge/discharge and constrained power, waypoint flight, connector API plus explicitly reported dock/manual coverage.
- [x] Use bounds/timeouts and cleanup. Preserve complete report across reload; refuse silent truncation.
- [x] Add console run command and export the simulated LCD; verify end-to-end run.

## Task 4: Comparison, deployment and operating instructions

Files: FleetCommand.Validation.Launcher/ReportComparison.cs, tools/Deploy-Validation.ps1, docs/validation.md, solution/project wiring.

- [x] Add failing comparison tests for identical, changed, incomplete, nonfinite and mismatched reports.
- [x] Implement strict schema/case/preflight validation and documented provisional channel tolerances.
- [x] Document commands, block coverage, world setup, complete LCD extraction and residual manual tests.
- [x] Run full suite, MDK build, launcher and self/negative comparison; inspect artifacts and complete final code review.
- [x] Deploy reviewed artifacts separately to game folders if permissions permit; never claim live-game equivalence without a real report.

## Execution rulings

- User explicitly requested native execution through completion; no additional plan approval gate.
- Work in the existing feature/test-framework checkout to keep the deliverable in the shared workspace. No merge/push or unrelated cleanup.
- Baseline excludes the two known AppData-dependent fixtures until task 1 replaces them.
- Initial baseline test invocation hit NUnit's default output path outside the workspace; rerun with explicit results/work directory before assessing test health.
