using System;
using FleetCommand.Test.Observations;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// The primary test surface (): owns the isolated world,
    /// deterministic clock, update scheduler, and trace recorders, and
    /// forwards scenario controls to it. One context per test; disposed by the
    /// fixture teardown.
    /// </summary>
    public sealed class FleetTestContext : IDisposable
    {
        private bool _disposed;

        /// <summary>The isolated simulation world.</summary>
        public SimWorld World { get; }

        /// <summary>The current tick (monotonic).</summary>
        public long Tick => World.Clock.Tick;

        /// <summary>Simulation time in seconds (an exact multiple of 1/60).</summary>
        public double Time => World.Clock.Time;

        internal FleetTestContext(SimWorld world)
        {
            World = world;
        }

        /// <summary>
        /// Spawns a ship: builds a fresh rigid body (compound shape + combined
        /// mass from the template's physical blocks), inserts it into the
        /// world at the given <paramref name="initialState"/>, creates fresh
        /// block state (one terminal or functional block per template block,
        /// registered with the grid's terminal system), and returns a
        /// <see cref="SimShip"/> handle whose grid owns the body. Each spawn
        /// creates independent block state and fresh Havok objects; mutating
        /// one ship never mutates its template or another instance ().
        /// </summary>
        public SimShip Spawn(ShipTemplate template, in SpawnState initialState)
        {
            if (template == null)
            {
                throw new ArgumentNullException(nameof(template));
            }

            long id = World.NextEntityId();
            SimRigidBodyState bodyState = initialState.ToBodyState(id);
            SimRigidBody body = SimRigidBody.Create(
                id, template, World.ScenarioOrigin, in bodyState, World.Profile);
            World.AddBody(body);

            var grid = new SimCubeGrid(id, template.Name, template.GridSize, body, World);
            SimShipController controller = null;
            foreach (TemplateBlock templateBlock in template.Blocks)
            {
                SimTerminalBlock block = CreateBlock(World, templateBlock, grid);
                grid.AddBlock(block);
                SimFunctionalBlock functional = block as SimFunctionalBlock;
                if (functional != null)
                {
                    World.RegisterFunctionalBlock(functional);
                }
                if (controller == null)
                {
                    controller = block as SimShipController;
                }
            }
            World.AddGrid(grid);

            return new SimShip(id, grid, controller);
        }

        /// <summary>
        /// Instantiates the simulator block matching a template block's kind.
        /// Stages 4–5 map the kinds onto the hierarchy bases: standard
        /// blocks become <see cref="SimTerminalBlock"/>; fixed thrusters
        /// become <see cref="SimThrust"/> (functional blocks in the game,
        /// ); the ship controller stack and the programmable
        /// block / text panel become functional leaves.
        /// </summary>
        private static SimTerminalBlock CreateBlock(SimWorld world, TemplateBlock templateBlock, SimCubeGrid grid)
        {
            long blockId = world.NextEntityId();
            switch (templateBlock.Kind)
            {
                case BlockKind.FixedThruster:
                    return new SimThrust(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.ShipController:
                    return new SimShipController(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.RemoteControl:
                    return new SimRemoteControl(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.Cockpit:
                    return new SimCockpit(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.ProgrammableBlock:
                    return new SimProgrammableBlock(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.TextPanel:
                    return new SimTextPanel(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.Battery:
                    return new SimBatteryBlock(blockId, templateBlock.Name, grid, templateBlock);
                default:
                    return new SimTerminalBlock(blockId, templateBlock.Name, grid, templateBlock);
            }
        }

        /// <summary>Advances the simulation by one fixed 1/60 s step.</summary>
        public void Step()
        {
            World.Step();
        }

        /// <summary>Advances the simulation by <paramref name="ticks"/> fixed steps.</summary>
        public void Step(int ticks)
        {
            World.Step(ticks);
        }

        /// <summary>Runs for <paramref name="seconds"/> of simulation time.</summary>
        public void RunFor(double seconds)
        {
            World.RunFor(seconds);
        }

        /// <summary>
        /// Steps until <paramref name="condition"/> (evaluated against this
        /// context) is satisfied by a completed-state observation, or
        /// <paramref name="timeoutSeconds"/> elapses. Stops at the first
        /// tick whose completed-state observation satisfies the predicate;
        /// returns false on timeout.
        /// </summary>
        public bool RunUntil(Func<FleetTestContext, bool> condition, double timeoutSeconds)
        {
            return World.RunUntil(() => condition(this), timeoutSeconds);
        }

        /// <summary>Schedules a scenario mutation to run on <paramref name="tick"/>.</summary>
        public void AtTick(long tick, Action<FleetTestContext> action)
        {
            World.AtTick(tick, () => action(this));
        }

        /// <summary>Registers a per-Update1 callback with start-of-tick state.</summary>
        public void EveryUpdate1(Action<SimUpdateContext> action)
        {
            World.EveryUpdate1(action);
        }

        /// <summary>
        /// Requests a bounded time-series recorder. Strictly arrivals in
        /// ; until then this member reports itself unsupported
        /// rather than silently recording nothing.
        /// </summary>
        public TraceRecorder<T> Record<T>(Func<FleetTestContext, T> sample)
        {
            throw new NotSupportedException(
                "FleetTestContext.Record is not implemented until  (observations) lands.");
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }
            World.Dispose();
            _disposed = true;
        }
    }
}
