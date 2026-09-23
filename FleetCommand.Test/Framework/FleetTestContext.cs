using System;
using FleetCommand.Test.Observations;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// The primary test surface: owns the isolated world, deterministic clock, update
    /// scheduler, and trace recorders, and forwards scenario controls to it. One per test.
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
        /// Spawns a ship: builds a fresh rigid body and block state from
        /// <paramref name="template"/>, inserts them into the world at
        /// <paramref name="initialState"/>, and returns a <see cref="SimShip"/>.
        /// Each spawn has independent state; mutating one never mutates its template or another.
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
            // The designated controller when the template names one; otherwise the first
            // controller-kind block.
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
                if (templateBlock.IsControllerKind)
                {
                    bool designated = template.DesignatedControllerName != null
                        && string.Equals(template.DesignatedControllerName, templateBlock.Name, StringComparison.Ordinal);
                    if (designated || (template.DesignatedControllerName == null && controller == null))
                    {
                        controller = block as SimShipController;
                    }
                }
            }
            World.AddGrid(grid);

            return new SimShip(id, grid, controller);
        }

        /// <summary>
        /// Instantiates the simulator block matching a template block's kind: standard blocks
        /// become <see cref="SimTerminalBlock"/>; fixed thrusters become <see cref="SimThrust"/>
        /// (functional blocks in the game); controllers and the PB / text panel are functional
        /// leaves.
        /// </summary>
        private static SimTerminalBlock CreateBlock(SimWorld world, TemplateBlock templateBlock, SimCubeGrid grid)
        {
            long blockId = world.NextEntityId();
            switch (templateBlock.Kind)
            {
                case BlockKind.FixedThruster:
                    return new SimThrust(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.Gyro:
                    return new SimGyro(blockId, templateBlock.Name, grid, templateBlock);
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
                case BlockKind.Reactor:
                    return new SimReactorBlock(blockId, templateBlock.Name, grid, templateBlock);
                case BlockKind.Connector:
                    return new SimShipConnector(blockId, templateBlock.Name, grid, templateBlock);
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
        /// Requests a bounded time-series recorder sampling <paramref name="sample"/> once per
        /// tick, retaining at most <see cref="TraceRecorder{T}.Capacity"/> samples (oldest
        /// dropped). Nothing is recorded until this is called.
        /// </summary>
        public TraceRecorder<T> Record<T>(Func<FleetTestContext, T> sample)
        {
            return Record(sample, DefaultRecorderCapacity);
        }

        /// <summary>Requests a bounded time-series recorder with a caller-specified
        /// <paramref name="capacity"/>.</summary>
        public TraceRecorder<T> Record<T>(Func<FleetTestContext, T> sample, int capacity)
        {
            var recorder = new TraceRecorder<T>(sample, this, capacity);
            World.RegisterSampler(recorder);
            return recorder;
        }

        /// <summary>The default time-series recorder capacity.</summary>
        public const int DefaultRecorderCapacity = 1024;

        // — scheduled mutation surface —

        /// <summary>Enables or disables a functional block.</summary>
        public void SetBlockEnabled(SimFunctionalBlock block, bool enabled)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            block.Enabled = enabled;
        }

        /// <summary>Damages (false) or repairs (true) a functional block.</summary>
        public void SetBlockFunctional(SimFunctionalBlock block, bool functional)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            block.IsFunctional = functional;
        }

        /// <summary>Enables or disables the named functional block on
        /// <paramref name="ship"/>.</summary>
        public void SetBlockEnabled(SimShip ship, string name, bool enabled)
        {
            SetBlockEnabled(ResolveFunctional(ship, name), enabled);
        }

        /// <summary>Damages or repairs the named functional block on
        /// <paramref name="ship"/>.</summary>
        public void SetBlockFunctional(SimShip ship, string name, bool functional)
        {
            SetBlockFunctional(ResolveFunctional(ship, name), functional);
        }

        /// <summary>
        /// Adds a block to <paramref name="ship"/> from a template block at runtime: creates
        /// fresh block state, registers it with the grid (terminal system, power bus), and
        /// rebuilds the body mass to include it. Returns the new block.
        /// </summary>
        public SimTerminalBlock AddBlock(SimShip ship, TemplateBlock templateBlock)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            if (templateBlock == null)
            {
                throw new ArgumentNullException(nameof(templateBlock));
            }
            SimCubeGrid grid = ship.Grid;
            SimTerminalBlock block = CreateBlock(World, templateBlock, grid);
            grid.AddBlock(block);
            SimFunctionalBlock functional = block as SimFunctionalBlock;
            if (functional != null)
            {
                World.RegisterFunctionalBlock(functional);
            }
            grid.RebuildBodyMass();
            return block;
        }

        /// <summary>
        /// Removes the named block from <paramref name="ship"/> at runtime: deregisters it,
        /// disposes it, and rebuilds the body mass. A removed connector is disconnected first.
        /// </summary>
        public void RemoveBlock(SimShip ship, string name)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            IMyTerminalBlock found = ship.TerminalSystem.GetBlockWithName(name);
            SimTerminalBlock block = found as SimTerminalBlock;
            if (block == null)
            {
                throw new ArgumentException(
                    "No block named '" + name + "' is present on ship '" + ship.Grid.CustomName + "' to remove.", nameof(name));
            }
            ship.Grid.RemoveBlock(block);
        }

        /// <summary>Changes the grid's available electrical ratio. See
        /// <see cref="SpaceEngineers.SimPowerBus.AvailableRatio"/>.</summary>
        public void SetPowerAvailableRatio(SimShip ship, float ratio)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            ship.Grid.PowerBus.AvailableRatio = ratio;
        }

        /// <summary>Changes the named reactor's remaining fuel (in item-count units). See
        /// <see cref="SpaceEngineers.SimReactorBlock.FuelRemaining"/>.</summary>
        public void SetReactorFuel(SimShip ship, string reactorName, float fuel)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            SimReactorBlock reactor = ship.TerminalSystem.GetBlockWithName(reactorName) as SimReactorBlock;
            if (reactor == null)
            {
                throw new ArgumentException("No reactor named '" + reactorName + "' on ship '" + ship.Grid.CustomName + "'.", nameof(reactorName));
            }
            reactor.FuelRemaining = fuel;
        }

        /// <summary>Changes the world's atmosphere effectiveness (partial).</summary>
        public void SetAtmosphereEffectiveness(float ratio)
        {
            World.SetAtmosphereEffectiveness(ratio);
        }

        /// <summary>Changes the world's natural gravity.</summary>
        public void SetGravity(SimGravityField gravity)
        {
            World.SetGravity(gravity);
        }

        /// <summary>Changes a block's inventory mass and rebuilds the body mass.</summary>
        public void SetInventoryMass(SimTerminalBlock block, float mass)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            block.InventoryMass = mass;
            block.Grid.RebuildBodyMass();
        }

        /// <summary>Skips the next Update1 callback without skipping physics — models a missed
        /// update.</summary>
        public void SkipNextUpdate1()
        {
            World.SuppressNextUpdate1();
        }

        /// <summary>Replaces a guidance target snapshot and refreshes it.</summary>
        public void ReplaceTargetSnapshot(string name, in SimTargetSnapshot snapshot)
        {
            World.Targets.Set(name, snapshot, World.Clock.Tick);
        }

        /// <summary>Stops refreshing <paramref name="name"/> so its snapshot becomes
        /// stale.</summary>
        public void StopRefreshingTarget(string name)
        {
            World.Targets.StopRefreshing(name);
        }

        /// <summary>Reads the current target snapshot for <paramref name="name"/>.</summary>
        public bool TryGetTargetSnapshot(string name, out SimTargetSnapshot snapshot)
        {
            return World.Targets.TryGet(name, out snapshot);
        }

        /// <summary>Changes the demanded body-vector thrust while the craft is flying.</summary>
        public void SetGroupDemand(SimShip ship, Vector3 bodyDemand)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            ship.Grid.ForceAggregator.SetGroupDemand(bodyDemand);
        }

        /// <summary>Changes the demanded attitude torque while the craft is turning.</summary>
        public void SetControlTorque(SimShip ship, Vector3 torque)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            ship.Grid.GyroRateSystem.ControlTorque = torque;
        }

        /// <summary>Applies an external world-frame linear impulse to the ship's body.</summary>
        public void ApplyLinearImpulse(SimShip ship, Vector3 impulse)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            ship.Grid.Body.ApplyLinearImpulse(impulse);
        }

        /// <summary>Applies an external world-frame angular impulse to the ship's body.</summary>
        public void ApplyAngularImpulse(SimShip ship, Vector3 impulse)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            ship.Grid.Body.ApplyAngularImpulse(impulse);
        }

        /// <summary>Applies an external world impulse at an off-COM world point.</summary>
        public void ApplyPointImpulse(SimShip ship, Vector3 impulse, Vector3D worldPoint)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            ship.Grid.Body.ApplyPointImpulse(impulse, worldPoint);
        }

        private static SimFunctionalBlock ResolveFunctional(SimShip ship, string name)
        {
            if (ship == null)
            {
                throw new ArgumentNullException(nameof(ship));
            }
            SimFunctionalBlock block = ship.TerminalSystem.GetBlockWithName(name) as SimFunctionalBlock;
            if (block == null)
            {
                throw new ArgumentException(
                    "No functional block named '" + name + "' is present on ship '" + ship.Grid.CustomName + "'.", nameof(name));
            }
            return block;
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
