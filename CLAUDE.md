# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FleetCommand is a Space Engineers Programmable Block script that provides autonomous ship autopilot and fleet coordination capabilities. Scripts run inside the game's sandboxed C# environment with strict per-run instruction limits (~50,000 instructions, ~10,000 method calls).

## Build System

This project uses MDK (Mod Development Kit) for Space Engineers programmable block development.

```powershell
# Build the project (produces binaries in FleetCommand/bin/x64/Debug/netframework48/)
dotnet build FleetCommand.sln

# Package the script for in-game use
# The packager outputs a single .cs file with all code in the IngameScript namespace
dotnet pack
```

The packager (`Mal.Mdk2.PbPackager`) strips namespaces and produces a single script file suitable for pasting into a Programmable Block's editor.

## Code Architecture

### Programmable Block Constraints

The game's programmable block extracts a single class named `Program` inheriting from `MyGridProgram` from `FleetCommand/Program.cs`. All code must live in the `IngameScript` namespace. This is a hard platform constraint.

### Shared Projects for Reusable Library Code

Code shared across multiple programmable blocks lives in a Visual Studio Shared Project (e.g., `FleetCommand.Test/`). The `.shproj` uses the standard VS CodeSharing targets and shares source files via the `.projitems` manifest. The main `FleetCommand.csproj` references the shared project to include its sources at compile time.

### The Guidance-Navigation-Control Pipeline

The system follows a layered pipeline (documented in `autopilot_spec.md`):

1. **Navigation** — Estimates craft state (position, velocity, angular velocity) and tracks external contacts via filtering
2. **Guidance** — Active behavior emits desired world acceleration + attitude mask + time-to-go
3. **Collision Avoidance** — Warps commanded acceleration against world model (external to allocation)
4. **Control Allocation** — Resolves attitude and per-axis thrust via fixed priority (feasibility → mask → optimizer)
5. **Control Loops** — Drive gyros and thrusters to hit setpoints
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
- Gyro override is a rate controller — sets torque = desired_accel × J, clamped to gyro ceiling
- Angular inertia J is NOT exposed by the API — must identify online from saturated gyro response

### Block Enumeration Discipline

- Cache all block references once on construction/recompile
- Never re-fetch blocks inside the per-tick loop
- Keep custom-data parsing and display strings off the hot path
- Control loops can run at 60 Hz; guidance and world-model at 10 Hz if budget is tight

## Key Files

| File | Purpose |
|------|---------|
| `autopilot_spec.md` | Full mathematical specification of the autopilot system |
| `FleetCommand/Program.cs` | Programmable block entry point and implementation |
| `FleetCommand/FleetCommand.csproj` | Project file with MDK packages and build config |
| `FleetCommand/mdk.ini` | MDK settings (minification, namespace, ignore patterns) |

## Runtime Model

This is **not** a normal C# application. It is a single Programmable Block (PB) script compiled inside the Space Engineers sandbox. The rules below override ordinary C# project conventions.

In game, the PB compiles your code as the **body of one class**:

```csharp
public sealed class Program : MyGridProgram { /* ALL your code goes here */ }
```

Consequences:
- There is exactly **one class** (`Program`) and effectively **one namespace**
- Any helper class becomes a **nested class of `Program`**
- C# nested classes get **no implicit reference** to the enclosing `Program` instance — they cannot see `Echo`, `GridTerminalSystem`, etc.

## Hard Rules

1. **Namespace.** Root namespace is `FleetCommand`. Each project gets its own sub-namespace: `FleetCommand.Autopilot`, `FleetCommand.Common`, etc. All source files declare their project's namespace.

2. **Everything lives in `partial class Program : MyGridProgram`.** Split code across multiple files freely, but each file declares the *same* `partial class Program : MyGridProgram` in the *same* namespace. Do not put free-standing top-level classes in their own namespaces.

3. **Helper classes are nested inside `Program`** and receive PB services by constructor injection. They never reference PB services as if global.

4. **PB services are instance members of `MyGridProgram`, not globals.** Available *only* inside `Program`'s own members:

   | member | type |
   |---|---|
   | `GridTerminalSystem` | `IMyGridTerminalSystem` |
   | `Me` | `IMyProgrammableBlock` |
   | `Runtime` | `IMyGridProgramRuntimeInfo` |
   | `Echo` | `Action<string>` (call it like a method) |
   | `Storage` | `string` |
   | `IGC` | `IMyIntergridCommunicationSystem` |

   To use any of these from a helper class, **pass the `Program` (`this`) into the helper's constructor** and call through that reference. Do not cache `Echo` as a delegate — the PB reassigns it each run; call `_p.Echo(...)`.

5. **Entry points only:** `public Program()` (constructor — cache blocks, set `Runtime.UpdateFrequency` here), `public void Save()`, and `public void Main(string argument, UpdateType updateSource)`. Schedule ticks with `Runtime.UpdateFrequency` (`Update1` / `Update10` / `Update100`); do not spin or sleep.

6. **Sandbox whitelist.** No `System.IO`, `System.Console`, threads, `Task`, `async`/`await`, reflection, `DateTime.Now` (use `Runtime.TimeSinceLastRun`), or non-whitelisted APIs. The MDK² analyzer flags violations as build errors — respect them rather than suppressing.

7. **Performance.** Resolve and **cache all block references in the constructor**; never call `GridTerminalSystem.GetBlocksOf...` inside the per-tick path. Reuse `List<T>` buffers; avoid per-tick allocations and LINQ in hot loops. The PB has a per-run instruction limit (~50k executed instructions); the math here is cheap, block enumeration and string building are not.

## Canonical Skeleton

```csharp
using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand
{
    partial class Program : MyGridProgram
    {
        readonly Autopilot _autopilot;

        public Program()
        {
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
            _autopilot = new Autopilot(this);          // inject Program
        }

        public void Save() { }

        public void Main(string argument, UpdateType updateSource)
        {
            _autopilot.Tick();
        }

        // Helper nested in Program; gets PB services via the injected reference.
        public class Autopilot
        {
            readonly Program _p;
            readonly List<IMyGyro> _gyros = new List<IMyGyro>();

            public Autopilot(Program p)
            {
                _p = p;
                _p.GridTerminalSystem.GetBlocksOfType(_gyros);   // cache once
            }

            public void Tick()
            {
                double dt = _p.Runtime.TimeSinceLastRun.TotalSeconds;
                _p.Echo($"gyros: {_gyros.Count}, dt: {dt:0.000}");
            }
        }
    }
}
```

## MDK² Notes

- The PbPackager finds the `Program` class, flattens it, and **strips the namespace** on deploy — so `namespace FleetCommand` is for in-editor consistency only
- Build output goes to the configured SE local script folder via `.mdk.ini`

## Commit Style

Use conventional commits: lowercase subject, no trailing period, blank line before body. Examples:
- `feat: add entry PB project`
- `fix: resolve gyro saturation drift`

## API Reference Location

Game API interfaces are defined in `../SpaceEngineers/`:
- `Sandbox.ModAPI.Ingame` — public ingame interfaces (`IMyShipController`, `IMyThrust`, `IMyGyro`, etc.)
- `Sandbox.Game/Entities/MyShipController.cs` — ship controller implementation with velocity and mass methods
- `Sandbox.Game/Entities/MyGyro.cs` — gyro implementation with `MaxGyroForce` property