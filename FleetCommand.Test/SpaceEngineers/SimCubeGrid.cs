using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using Havok;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The rigid grid: owns its <see cref="SimRigidBody"/> and its blocks, and implements
    /// the real <see cref="IMyCubeGrid"/> from VRage.Game plus its <see cref="IMyEntity"/> parent.
    /// The grid's world transform is owned by the body; block membership, position queries, and
    /// same-grid / construct identity are backed by this grid's block list. This stage inverts the
    /// scene: <c>SimCubeGrid</c> is the grid that owns blocks.</summary>
    public sealed class SimCubeGrid : SimEntity, IMyCubeGrid
    {
        private readonly Dictionary<Vector3I, SimTerminalBlock> _blockByPosition = new Dictionary<Vector3I, SimTerminalBlock>();

        /// <summary>The rigid body this grid owns (one per dynamic grid).</summary>
        public SimRigidBody Body { get; }

        /// <summary>The world that owns this grid (gravity source, tick clock, …).</summary>
        public SimWorld World { get; }

        /// <summary>Grid cell size in meters.</summary>
        public float GridSize { get; }

        /// <summary>The grid terminal system used to discover this grid's blocks.</summary>
        public SimGridTerminalSystem TerminalSystem { get; }

        /// <summary>The grid's deterministic power bus (the ship's <c>SimShip.Power</c>):
        /// functional blocks register as consumers and batteries as supply when added to the grid;
        /// the world recomputes it in tick phase 4.</summary>
        public SimPowerBus PowerBus { get; }

        /// <summary>The grid's thrust aggregation step: the world calls
        /// <see cref="ForceAggregator.Aggregate"/> in phase 5,
        /// <see cref="ForceAggregator.ApplyThrust"/> in phase 6 (one resultant body-space linear
        /// impulse at the grid COM), and <see cref="ForceAggregator.UpdateThrusterTelemetry"/> in
        /// phase 8 (thrusters register when added to the grid).</summary>
        public ForceAggregator ForceAggregator { get; }

        /// <summary>The grid's gyro-rate system: the world calls
        /// <see cref="GyroRateSystem.Aggregate"/> in phase 5,
        /// <see cref="GyroRateSystem.ApplyTorque"/> in phase 6 (one resultant body-space angular
        /// impulse), and <see cref="GyroRateSystem.UpdateGyroTelemetry"/> in phase 8 (gyros
        /// register when added to the grid).</summary>
        public GyroRateSystem GyroRateSystem { get; }

        /// <summary>The grid's controller-driven base thrust demand: the world calls
        /// <see cref="BaseControlSystem.ComputeBaseDemand"/> in phase 5 (before
        /// <see cref="ForceAggregator.Aggregate"/>) to run the occupied controllers' MoveIndicator
        /// + inertia dampening + anti-gravity through the game's <c>ComputeBaseThrust</c> and set
        /// the aggregator's group demand. Controllers register when added to the grid.</summary>
        public BaseControlSystem BaseControl { get; }

        /// <summary>The grid's controller rotation control: the world calls
        /// <see cref="RotationControlSystem.ComputeControlTorque"/> in phase 5 to convert
        /// under-control controllers' RotationIndicator/RollIndicator into
        /// <see cref="GyroRateSystem.ControlTorque"/> (the game's
        /// <c>MyShipController.MoveAndRotate</c> :1816-1824). Controllers register when added to
        /// the grid.</summary>
        public RotationControlSystem RotationControl { get; }

        /// <summary>The grid's remote-control autopilot: the world calls
        /// <see cref="AutopilotSystem.ComputeAutopilot"/> in phase 5 to fly an enabled remote
        /// toward its waypoint through the same <see cref="GyroRateSystem.ControlTorque"/> channel
        /// and the <see cref="ForceAggregator"/> AI thrust path. Remotes register when added to the
        /// grid.</summary>
        public AutopilotSystem Autopilot { get; }

        /// <summary>Also the grid's user-visible terminal name (settable).</summary>
        public string CustomName { get; set; }

        /// <summary>World-visible position of the grid origin (from the body).</summary>
        public Vector3D Position => Body.PositionWorld;

        public SimCubeGrid(long entityId, string name, float gridSize, SimRigidBody body, SimWorld world)
            : base(entityId, name)
        {
            if (body == null)
            {
                throw new ArgumentNullException(nameof(body));
            }
            if (world == null)
            {
                throw new ArgumentNullException(nameof(world));
            }
            Body = body;
            World = world;
            GridSize = gridSize;
            CustomName = name;
            TerminalSystem = new SimGridTerminalSystem(this);
            PowerBus = new SimPowerBus(this);
            ForceAggregator = new ForceAggregator(this);
            GyroRateSystem = new GyroRateSystem(this);
            BaseControl = new BaseControlSystem(this);
            RotationControl = new RotationControlSystem(this);
            Autopilot = new AutopilotSystem(this);
        }

        /// <inheritdoc />
        public override MatrixD WorldMatrix => Body.WorldMatrix;

        /// <summary>The world-&gt;grid rotation matrix (R^T) for transforming vectors from the
        /// world frame into the grid (body) frame — the game's <c>WorldMatrixNormalizedInv</c>
        /// rotation part used at every world-to-local control read.</summary>
        public Matrix WorldToLocal => Matrix.Transpose(((Matrix)WorldMatrix).GetOrientation());

        /// <inheritdoc />
        public override string DisplayName => CustomName;

        // — IMyCubeGrid —

        /// <inheritdoc />
        public MyCubeSize GridSizeEnum => GridSize <= 1.5f ? MyCubeSize.Small : MyCubeSize.Large;

        /// <inheritdoc />
        public bool IsStatic => !Body.IsDynamic;

        /// <inheritdoc />
        public Vector3 LinearVelocity => (Vector3)Body.LinearVelocity;

        /// <inheritdoc />
        public float Speed => (float)Body.LinearVelocity.Length();

        /// <summary>Grid cell bounds: the minimum block cell across all blocks.</summary>
        public Vector3I Min
        {
            get
            {
                Vector3I? min = null;
                IReadOnlyList<SimTerminalBlock> blocks = TerminalSystem.Blocks;
                for (int i = 0; i < blocks.Count; i++)
                {
                    min = min.HasValue ? MinComponents(min.Value, blocks[i].Min) : blocks[i].Min;
                }
                return min ?? Vector3I.Zero;
            }
        }

        /// <summary>Grid cell bounds: the maximum block cell across all blocks.</summary>
        public Vector3I Max
        {
            get
            {
                Vector3I? max = null;
                IReadOnlyList<SimTerminalBlock> blocks = TerminalSystem.Blocks;
                for (int i = 0; i < blocks.Count; i++)
                {
                    max = max.HasValue ? MaxComponents(max.Value, blocks[i].Max) : blocks[i].Max;
                }
                return max ?? Vector3I.Zero;
            }
        }

        /// <inheritdoc />
        public bool CubeExists(Vector3I pos)
        {
            return _blockByPosition.ContainsKey(pos);
        }

        /// <inheritdoc />
        public IMySlimBlock GetCubeBlock(Vector3I pos)
        {
            SimTerminalBlock block;
            if (_blockByPosition.TryGetValue(pos, out block))
            {
                return block.Closed ? null : new SimSlimBlock(block);
            }
            return null;
        }

        /// <inheritdoc />
        public Vector3D GridIntegerToWorld(Vector3I gridCoords)
        {
            // Grid cell origin transformed by the grid's world matrix —
            // the game's grid-coordinate to world-space conversion.
            Vector3D local = new Vector3D(gridCoords.X, gridCoords.Y, gridCoords.Z) * GridSize;
            return Vector3D.Transform(local, WorldMatrix);
        }

        /// <inheritdoc />
        public Vector3I WorldToGridInteger(Vector3D coords)
        {
            MatrixD world = WorldMatrix;
            MatrixD inverse;
            MatrixD.Invert(ref world, out inverse);
            Vector3D local = Vector3D.Transform(coords, inverse) / (double)GridSize;
            return Vector3I.Round(local);
        }

        /// <inheritdoc />
        public bool IsSameConstructAs(IMyCubeGrid other)
        {
            // For a single rigid grid, "same construct" equals "same grid";
            // there are no rotors or pistons to follow yet.
            return other != null && other.EntityId == EntityId;
        }

        // — block membership (sim surface) —

        /// <summary>Registers a spawned block with this grid and its terminal system.</summary>
        internal void AddBlock(SimTerminalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            _blockByPosition.Add(block.Position, block);
            TerminalSystem.RegisterBlock(block);
            SimFunctionalBlock functional = block as SimFunctionalBlock;
            if (functional != null)
            {
                PowerBus.RegisterConsumer(functional);
            }
            SimBatteryBlock battery = block as SimBatteryBlock;
            if (battery != null)
            {
                PowerBus.RegisterSupply(battery);
            }
            SimReactorBlock reactor = block as SimReactorBlock;
            if (reactor != null)
            {
                PowerBus.RegisterProducer(reactor);
            }
            SimThrust thruster = block as SimThrust;
            if (thruster != null)
            {
                ForceAggregator.RegisterThruster(thruster);
            }
            SimGyro gyro = block as SimGyro;
            if (gyro != null)
            {
                GyroRateSystem.RegisterGyro(gyro);
            }
            SimShipConnector connector = block as SimShipConnector;
            if (connector != null)
            {
                World.RegisterConnector(connector);
            }
            SimShipController controller = block as SimShipController;
            if (controller != null)
            {
                BaseControl.RegisterController(controller);
                RotationControl.RegisterController(controller);
            }
            SimRemoteControl remote = block as SimRemoteControl;
            if (remote != null)
            {
                Autopilot.RegisterRemote(remote);
            }
        }

        /// <summary>Removes a block from this grid: disconnects it if a connector, deregisters it
        /// from every system it joined on add (terminal system, power bus, force aggregator, gyro
        /// system, base / rotation control, autopilot, world registries), removes it from the
        /// position map, disposes it, and rebuilds the body's mass. Idempotent for an
        /// already-closed block.</summary>
        public void RemoveBlock(SimTerminalBlock block)
        {
            if (block == null)
            {
                throw new ArgumentNullException(nameof(block));
            }
            if (block.Grid != this || block.Closed)
            {
                return;
            }

            SimShipConnector connector = block as SimShipConnector;
            if (connector != null)
            {
                if (connector.ConnectedInternal || connector.ApproachingInternal)
                {
                    connector.Disconnect();
                }
                World.UnregisterConnector(connector);
            }

            TerminalSystem.DeregisterBlock(block);
            _blockByPosition.Remove(block.Position);

            SimFunctionalBlock functional = block as SimFunctionalBlock;
            if (functional != null)
            {
                PowerBus.UnregisterConsumer(functional);
                World.UnregisterFunctionalBlock(functional);
            }
            SimBatteryBlock battery = block as SimBatteryBlock;
            if (battery != null)
            {
                PowerBus.UnregisterBattery(battery);
            }
            SimReactorBlock reactor = block as SimReactorBlock;
            if (reactor != null)
            {
                PowerBus.UnregisterProducer(reactor);
            }
            SimThrust thruster = block as SimThrust;
            if (thruster != null)
            {
                ForceAggregator.UnregisterThruster(thruster);
            }
            SimGyro gyro = block as SimGyro;
            if (gyro != null)
            {
                GyroRateSystem.UnregisterGyro(gyro);
            }
            SimShipController controller = block as SimShipController;
            if (controller != null)
            {
                BaseControl.UnregisterController(controller);
                RotationControl.UnregisterController(controller);
            }
            SimRemoteControl remote = block as SimRemoteControl;
            if (remote != null)
            {
                Autopilot.UnregisterRemote(remote);
            }

            block.Dispose();
            RebuildBodyMass();
        }

        /// <summary>Rebuilds the grid body's mass properties from the current live block set.
        /// Mirrors <c>MyPhysicsBody.UpdateMassProps</c> — combines the box mass elements with Havok
        /// and applies via <c>RigidBody.SetMassProperties</c> — so total mass, COM, and inertia
        /// ground truth follow the live block inventory.</summary>
        internal void RebuildBodyMass()
        {
            var elements = new List<HkMassElement>();
            IReadOnlyList<SimTerminalBlock> blocks = TerminalSystem.Blocks;
            for (int i = 0; i < blocks.Count; i++)
            {
                SimTerminalBlock block = blocks[i];
                if (block.Closed)
                {
                    continue;
                }
                TemplateBlock template = block.Template;
                HkMassProperties box = HkInertiaTensorComputer.ComputeBoxVolumeMassProperties(
                    template.HalfExtents, template.DryMass);
                elements.Add(new HkMassElement
                {
                    Properties = box,
                    Tranform = Matrix.CreateTranslation(template.CenterLocal),
                });
                if (block.InventoryMass > 0f)
                {
                    HkMassProperties inventory = HkInertiaTensorComputer.ComputeBoxVolumeMassProperties(
                        template.HalfExtents, block.InventoryMass);
                    elements.Add(new HkMassElement
                    {
                        Properties = inventory,
                        Tranform = Matrix.CreateTranslation(template.CenterLocal),
                    });
                }
            }
            Body.UpdateMassProperties(MassProperties.FromMassElements(elements));
        }

        /// <summary>Disposes the grid's owned body and closes all its blocks. Idempotent;
        /// subsequent enumeration over the terminal system and the world's tick continue to work
        /// (closed items are skipped).</summary>
        public void Dispose()
        {
            if (IsDisposed)
            {
                return;
            }
            IReadOnlyList<SimTerminalBlock> blocks = TerminalSystem.Blocks;
            for (int i = 0; i < blocks.Count; i++)
            {
                blocks[i].Dispose();
            }
            if (Body != null)
            {
                Body.Dispose();
            }
            IsDisposed = true;
        }

        private static Vector3I MinComponents(Vector3I a, Vector3I b)
        {
            return new Vector3I(
                Math.Min(a.X, b.X),
                Math.Min(a.Y, b.Y),
                Math.Min(a.Z, b.Z));
        }

        private static Vector3I MaxComponents(Vector3I a, Vector3I b)
        {
            return new Vector3I(
                Math.Max(a.X, b.X),
                Math.Max(a.Y, b.Y),
                Math.Max(a.Z, b.Z));
        }
    }
}
