# FleetCommand for Space Engineers

FleetCommand is an early-stage fleet orchestration project for programmable
blocks. The current development foundation is a native simulation and game
conformance framework; the main fleet script remains a bootstrap prototype.

## Projects

| Project | Purpose |
| --- | --- |
| `FleetCommand` | Fleet programmable-block entry point |
| `FleetCommand.Common` | Shared script utilities |
| `FleetCommand.Test` | Native Havok simulator, ingame API implementations and NUnit tests |
| `FleetCommand.Validation` | Shared C# 6 experiments and MDK programmable-block entry point |
| `FleetCommand.Validation.Launcher` | Runs those same experiments in the simulator and exports LCD text |
| `FleetCommand.Validation.Tests` | Experiment, replay and report-comparison regressions |

## Build and test

Use Windows x64, a .NET SDK with .NET Framework 4.8 targeting support, and a local
Space Engineers installation. Game assemblies and native Havok libraries are
resolved from that installation, rather than distributed with this repository.
Create the ignored `MDK.paths.props` in the repository root:

```xml
<Project>
  <PropertyGroup>
    <SpaceEngineersBin>C:\Path\To\SpaceEngineers\Bin64</SpaceEngineersBin>
  </PropertyGroup>
</Project>
```

```powershell
dotnet restore FleetCommand.sln
dotnet test FleetCommand.sln --no-restore -p:MdkBuildConfiguration=PackExplicitly -m:1
```

The test command suppresses MDK packaging. To pack and check the validation
script explicitly:

```powershell
dotnet build FleetCommand.Validation/FleetCommand.Validation.csproj --no-restore -p:Platform=x64 -p:MdkBuildConfiguration=Debug -m:1
./tools/Test-PackedValidation.ps1
```

The packed script is written under
`artifacts/validation-script/FleetCommand.Validation/`. Deploy the script and
three named blueprints to local game AppData with `./tools/Deploy-Validation.ps1`.
Deployment verifies copies and backs up changed files; tests do not deploy.

## Game and simulator comparison

The [station mission](docs/station-validation.md) is the most complete current
experiment. Paste `FleetCommand_Validation_Station_v1`, load the validation script
into `FC Validation`, and run `dockprobe` in empty, zero-gravity survival space.
It undocks, runs power and waypoint tests, returns, observes connector capture,
explicitly connects, and holds. Copy the complete text from both `FC Dock Report`
and `FC Report` after completion. Motion is measured relative to the starting
frame, so exact world placement is unnecessary.

Replay the checked-in game starting attitude with the same shared code:

```powershell
./FleetCommand.Validation.Launcher/bin/x64/Debug/net48/FleetCommand.Validation.Launcher.exe dockprobe artifacts/station-simulator.txt --frame-from validation-data/2026-10-19/game-station-dock-v1.txt
```

The launcher exports the actual simulated LCD text to the requested path and
its `.flight.txt` sibling. Simulation runs without real-time pacing. See
[validation commands and schemas](docs/validation.md),
[flight probes](docs/flight-probe.md), and
[retained game measurements](validation-data/README.md).

## Conformance status

The current baseline passes 417 automated tests and has completed the station
mission in both game and simulator. Recorded power-limited velocity gains agree
within 0.000024 m/s; both waypoint probes finish stationary within 9 cm of their
targets. These measurements cover the tested vacuum fixtures and operating
conditions, rather than complete game equivalence.

Final connector seating differs by roughly 17 mm. Cross-grid power distribution
and terminal discovery remain incomplete. Planetary flight, general collision
geometry, IGC, and actual ingame instruction-budget enforcement are outside this
validated baseline. See the [source audit](docs/validation-source-audit-2026-10-19.md)
and [measured results](docs/station-validation.md#power-discrepancy-resolved-thrust-precedes-distribution).

Builds, native investigation probes, captured scratch logs, machine-local paths,
and decompiled game sources stay ignored. Only selected original game recordings
are retained as comparison data; generated simulator reports can be reproduced.
