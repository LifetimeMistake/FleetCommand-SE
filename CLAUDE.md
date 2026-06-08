# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FleetCommand is a modular fleet orchestration framework for Space Engineers Programmable Blocks. It provides autonomous ship autopilot, fleet coordination, and a protocol for vessels to communicate, expose services, and negotiate capabilities across a fleet.

Scripts run inside the game's sandboxed C# environment with strict per-run instruction limits (~50,000 instructions, ~10,000 method calls).

## Build System

```powershell
# Build the project (produces binaries in FleetCommand/bin/x64/Debug/netframework48/)
dotnet build FleetCommand.sln

# Package the script for in-game use
# The packager outputs a single .cs file with all code in the IngameScript namespace
dotnet pack
```

The `Mal.Mdk2.PbPackager` strips namespaces and produces a single script file suitable for pasting into a Programmable Block's editor.

## Project Architecture

### Source Tree vs. Packed Output

The source tree follows **standard .NET conventions**:
- Multiple namespaces (`FleetCommand.Common`, `FleetCommand.Autopilot`, etc.)
- Visual Studio Shared Projects (`.shproj`) for shared code
- Multiple projects referencing a common framework

MDK2 abstracts away SE's script constraints at **build time**:
- The packager extracts the `Program` class, strips namespaces, and bundles everything into a single `partial class Program : MyGridProgram` in `namespace IngameScript`
- `mdk.ini` `namespaces=IngameScript` controls the **output namespace only**, not development structure

### Core Concepts

| Concept | Role |
|---------|------|
| **Framework** | Reusable libraries (e.g., `FleetCommand.Autopilot`) providing core capabilities that programs use |
| **Protocol** | Common communication standard enabling fleet coordination and service negotiation |
| **Services** | RPC-server-like constructs exposing vessel capabilities to other fleet members. Every vessel may host different services. Services are negotiated via protocol — not static. |
| **Programs** | Local behaviors on a vessel. Use the framework, may host services, and implement mission logic. |

### Program as Entry Point

`Program` is the **entrypoint and bootstrapper only** — it does NOT contain core business logic.

Key rules:
- `Program` caches PB services and passes them as **individual references** to the services/components it creates
- `Program` never passes itself (`this`) to other classes
- This keeps the PB layer thin and decoupled from implementation

```csharp
public partial class Program : MyGridProgram
{
    readonly IMyGridTerminalSystem _gts;
    readonly Action<string> _echo;
    readonly Autopilot _autopilot;

    public Program()
    {
        _gts = GridTerminalSystem;
        _echo = Echo;
        _autopilot = new Autopilot(_gts, Runtime, IGC, _echo);
        Runtime.UpdateFrequency = UpdateFrequency.Update1;
    }

    public void Main(string argument, UpdateType updateSource)
    {
        _autopilot.Tick(Runtime.TimeSinceLastRun.TotalSeconds);
    }
}
```

## Project Structure

| Project | Type | Namespace | Purpose |
|---------|------|-----------|---------|
| `FleetCommand/` | Main PB entry | `IngameScript` (output) | Entry point. References shared projects. |
| `FleetCommand.Common/` | Shared Project | `FleetCommand.Common.*` | Utilities (Logger, RingBuffer, TimeSource) |
| `FleetCommand.Autopilot/` | (future) | `FleetCommand.Autopilot` | Reusable flight control library |

### Adding New Modules

New capability modules (Guidance, Navigation, Weapons, etc.) should follow the same pattern:
1. Create a new shared project or sub-namespace
2. Use standard .NET namespaces
3. Reference from `FleetCommand.csproj` via shared project import

## The Guidance-Navigation-Control Pipeline

The autopilot follows a layered pipeline (documented in `autopilot_spec.md`):

1. **Navigation** — Estimates craft state (position, velocity, angular velocity) and tracks external contacts via filtering
2. **Guidance** — Active behavior emits desired world acceleration + attitude mask + time-to-go
3. **Collision Avoidance** — Warps commanded acceleration against world model (external to allocation)
4. **Control Allocation** — Resolves attitude and per-axis thrust via fixed priority (feasibility → mask → optimizer). When a demand is infeasible the controller produces the *closest achievable force* — the Euclidean projection of the demand onto the thrust box — rather than scaling magnitude alone. This is what keeps an underactuated craft firing through a turn rather than coasting to zero thrust.
5. **Control Loops** — Drive gyros and thrusters to hit setpoints. The attitude loop emits a commanded angular velocity (not a position setpoint) carrying a demand-rate feedforward `ω_ff = d̂ × d̂_dot`, so the nose tracks a rotating demand without steady lag — the rotational counterpart of commanding acceleration rather than position. The translation loop realizes the closest achievable force at the craft's current attitude.
6. **Inertia Observer** — Passive online identification of rotational inertia from saturated gyro ticks

The key design contract between layers is **acceleration + attitude mask**, never waypoints. This decouples guidance from dynamics.

### Attitude Mask System

Guidance declares each rotational DOF as **bound** (specific value demanded) or **free** (controller may optimize). The allocation priority is:
1. Feasibility — craft must physically produce the acceleration
2. Mask — bound axes held if feasible
3. Optimizer — free axes oriented to maximize thrust along the demand

This enables underactuated craft (forward-only missiles) to properly prioritize: feasibility forces the nose along the demand, the mask's aim constraint yields, and the remaining free axes are optimized.

### Critical API Conventions (verified against game source)

- `IMyShipController.WorldMatrix` — rotation part is body-to-world transform R; columns are body axes in world coords
- `GetShipVelocities().AngularVelocity` — reported in **world frame**, must convert: `ω_B = Rᵀ · ω_W`
- `GetNaturalGravity()` — world-frame acceleration pointing down; fold into force demand as F* = m(a_cmd - g)
- Thruster capability is asymmetric — cache `MaxEffectiveThrust` sums per ±body axis as 6 numbers; represent as axis-aligned box F = [-Tx⁻, Tx⁺] × [-Ty⁻, Ty⁺] × [-Tz⁻, Tz⁺]
- Gyro override is a rate controller — accepts a commanded angular velocity `ω_cmd` and closes the torque loop internally at the torque ceiling. The controller's rotational contract is a commanded `ω_cmd`, not an attitude setpoint to be servoed. The attitude loop adds a demand-rate feedforward `ω_ff = d̂ × d̂_dot` so the nose tracks a rotating demand without steady lag.
- Angular inertia J is NOT exposed by the API — must identify online from saturated gyro response (section 9.3)

### Block Enumeration Discipline

- Cache all block references once on construction/recompile
- Never re-fetch blocks inside the per-tick loop
- Keep custom-data parsing and display strings off the hot path
- Control loops can run at 60 Hz; guidance and world-model at 10 Hz if budget is tight

## Key Files

| File | Purpose |
|------|---------|
| `autopilot_spec.md` | Full mathematical specification of the autopilot system |
| `FleetCommand/Program.cs` | Programmable block entry point (bootstrapper only) |
| `FleetCommand/FleetCommand.csproj` | Project file with MDK packages and build config |
| `FleetCommand/mdk.ini` | MDK settings (minification, namespace, ignore patterns) |
| `FleetCommand.Common/` | Shared library (Logger, RingBuffer, TimeSource) |

## Runtime Model

At **build time**, MDK2 bundles all source into a single `partial class Program : MyGridProgram` in `namespace IngameScript`. This is the runtime constraint of the packed output — not a development constraint.

At **development time**, the repo follows standard .NET conventions with multiple namespaces and shared projects.

## Hard Rules

1. **Standard .NET namespaces.** Use `FleetCommand.<Module>` for new modules. The packager handles namespace stripping at build time.

2. **Program is the entrypoint only.** It caches PB services and passes individual references (GTS, Runtime, IGC, etc.) to the components it creates. Never pass `this` to other classes.

3. **PB services are instance members of `MyGridProgram`.** Available inside `Program`'s own members:

   | member | type |
   |---|---|
   | `GridTerminalSystem` | `IMyGridTerminalSystem` |
   | `Me` | `IMyProgrammableBlock` |
   | `Runtime` | `IMyGridProgramRuntimeInfo` |
   | `Echo` | `Action<string>` (call it like a method) |
   | `Storage` | `string` |
   | `IGC` | `IMyIntergridCommunicationSystem` |

4. **Entry points only:** `public Program()` (constructor — cache blocks, set `Runtime.UpdateFrequency` here), `public void Save()`, and `public void Main(string argument, UpdateType updateSource)`. Schedule ticks with `Runtime.UpdateFrequency` (`Update1` / `Update10` / `Update100`); do not spin or sleep.

5. **Sandbox whitelist.** No `System.IO`, `System.Console`, threads, `Task`, `async`/`await`, reflection, `DateTime.Now` (use `Runtime.TimeSinceLastRun`), or non-whitelisted APIs. The MDK² analyzer flags violations as build errors — respect them rather than suppressing.

6. **Performance.** Resolve and **cache all block references in the constructor**; never call `GridTerminalSystem.GetBlocksOf...` inside the per-tick path. Reuse `List<T>` buffers; avoid per-tick allocations and LINQ in hot loops. The PB has a per-run instruction limit (~50k executed instructions); the math here is cheap, block enumeration and string building are not.

## MDK² Notes

- The PbPackager finds the `Program` class, flattens it, and **strips all namespaces** on deploy — so `namespace FleetCommand` is for in-editor consistency only
- Build output goes to the configured SE local script folder via `.mdk.ini`

### Script Size Limits

SE imposes ~100k character limit per script. Design modules to fit within this, or use separate script bundles that share the framework (different `.csproj` projects for different capability sets).

### Minification Options

Available in `mdk.ini` (`minify=`):
- `none` — No minification
- `trim` — Removes unused types (not members)
- `stripcomments` — trim + removes comments
- `lite` — stripcomments + removes whitespace
- `full` — lite + renames identifiers to short names

## Commit Style

Use conventional commits: lowercase subject, no trailing period, blank line before body. Examples:
- `feat: add entry PB project`
- `fix: resolve gyro saturation drift`

## API Reference Location

Game API interfaces are defined in `../SpaceEngineers/`:
- `Sandbox.ModAPI.Ingame` — public ingame interfaces (`IMyShipController`, `IMyThrust`, `IMyGyro`, etc.)
- `Sandbox.Game/Entities/MyShipController.cs` — ship controller implementation with velocity and mass methods
- `Sandbox.Game/Entities/MyGyro.cs` — gyro implementation with `MaxGyroForce` property