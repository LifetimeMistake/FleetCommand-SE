using System;
using System.Collections.Generic;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The ship control seat, implementing <see cref="IMyShipController"/> — the
    /// controller-facing surface the production autopilot adapter reads, so the frame conventions
    /// are load-bearing and match the game (source-verified: MyShipController.cs:3233-3238 returns
    /// linear AND angular velocity straight from the physics body with NO body-frame transform;
    /// :3212 returns world-frame natural gravity). <see cref="GetShipVelocities"/> is world-frame
    /// (adapters needing body-frame convert ω_B = Rᵀ·ω_W themselves);
    /// <see cref="GetNaturalGravity"/> returns the same world-frame vector the tick's phase 6
    /// applied to the body, so sign conventions (F* = m(a_cmd − g)) hold;
    /// <see cref="CenterOfMass"/> is in world coordinates. Implements the control flags
    /// (<see cref="DampenersOverride"/>, <see cref="ControlThrusters"/>), the scenario control
    /// inputs (<see cref="MoveIndicator"/>, <see cref="RotationIndicator"/>,
    /// <see cref="RollIndicator"/>), <see cref="CanControlShip"/>, <see cref="IsUnderControl"/>,
    /// <see cref="IsMainCockpit"/>, plus the block's inherited world pose; every other member
    /// throws a naming <see cref="NotSupportedException"/> (no silent defaults).</summary>
    public class SimShipController : SimFunctionalBlock, IMyShipController
    {
        private bool _isUnderControl;
        private bool _controlThrusters = true;
        private bool _controlGyros = true;
        private bool _isMainCockpit;
        private Vector3 _moveIndicator;
        private Vector2 _rotationIndicator;
        private float _rollIndicator;

        public SimShipController(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
        }

        /// <inheritdoc />
        public bool CanControlShip => IsWorking;

        /// <summary>Whether the ship is locally or remotely controlled. No player model exists in
        /// the simulator, so this is a scenario flag (default false).</summary>
        public bool IsUnderControl
        {
            get { return _isUnderControl; }
            set { _isUnderControl = value; }
        }

        /// <inheritdoc />
        public bool ControlThrusters
        {
            get { return _controlThrusters; }
            set { _controlThrusters = value; }
        }

        /// <summary>Whether the grid's inertial dampeners are on. This is the game's
        /// <c>IMyShipController.DampenersOverride</c>, which reads the SHARED per-grid dampener
        /// state (<c>EntityThrustComponent.DampenersEnabled</c>) and sets the whole grid's
        /// dampeners via <c>CubeGrid.EnableDampingInternal</c> (MyShipController.cs:770-786) — it
        /// is not a per-controller flag. All controllers on a grid share the grid's dampening
        /// state; defaulting on, matching the game (MyEntityThrustComponent.cs:463).</summary>
        public bool DampenersOverride
        {
            get
            {
                SimRigidBody body = Grid.Body;
                return body != null && !body.IsDisposed && body.DampenersEnabled;
            }
            set
            {
                SimRigidBody body = Grid.Body;
                if (body != null && !body.IsDisposed)
                {
                    body.DampenersEnabled = value;
                }
            }
        }

        /// <summary>Whether this controller drives the grid gyros (the game's
        /// <c>m_controlGyros</c>, which gates the whole rotation-control block in
        /// <c>MyShipController.MoveAndRotate</c> :1816-1824). Defaults on, matching the game's
        /// controller default.</summary>
        public bool ControlGyros
        {
            get { return _controlGyros; }
            set { _controlGyros = value; }
        }

        /// <summary>Directional control input (scenario-set; a plain stored value the autopilot may
        /// write or a test may observe). Raw — no frame convention is attached to it.</summary>
        public Vector3 MoveIndicator
        {
            get { return _moveIndicator; }
            set { _moveIndicator = value; }
        }

        /// <summary>Pitch/yaw control input (scenario-set).</summary>
        public Vector2 RotationIndicator
        {
            get { return _rotationIndicator; }
            set { _rotationIndicator = value; }
        }

        /// <summary>Roll control input (scenario-set).</summary>
        public float RollIndicator
        {
            get { return _rollIndicator; }
            set { _rollIndicator = value; }
        }

        /// <summary>Center of mass in world coordinates, from the grid body's local COM (game:
        /// IMyShipController.CenterOfMass → CubeGrid.Physics.CenterOfMassWorld).</summary>
        public Vector3D CenterOfMass
        {
            get
            {
                Vector3D local = new Vector3D(Grid.Body.CenterOfMassLocal);
                return Vector3D.Transform(local, Grid.WorldMatrix);
            }
        }

        /// <summary>True while this controller is the ship's main one. Setting it true demotes any
        /// other controller on the same grid, matching the property's contract ("setter checks if
        /// there is any other main cockpit on the ship before proceeding").</summary>
        public bool IsMainCockpit
        {
            get { return _isMainCockpit; }
            set
            {
                if (_isMainCockpit == value)
                {
                    return;
                }
                if (value)
                {
                    IReadOnlyList<SimTerminalBlock> blocks = Grid.TerminalSystem.Blocks;
                    for (int i = 0; i < blocks.Count; i++)
                    {
                        SimShipController other = blocks[i] as SimShipController;
                        if (other != null && !ReferenceEquals(other, this) && !other.Closed)
                        {
                            other._isMainCockpit = false;
                        }
                    }
                }
                _isMainCockpit = value;
            }
        }

        /// <summary>World-frame natural gravity at the ship's position — the same vector the tick's
        /// phase 6 applied to the body, so an adapter's sign convention (F* = m(a_cmd − g)) matches
        /// the game. Sampled at the body's world position at the current world gravity
        /// field.</summary>
        public Vector3D GetNaturalGravity()
        {
            Vector3 gravity = Grid.World.Gravity.GetNaturalGravity(Grid.Body.PositionWorld);
            return new Vector3D(gravity);
        }

        /// <summary>Artificial gravity at the current location. The simulator has no
        /// artificial-gravity generators, so this is genuinely zero.</summary>
        public Vector3D GetArtificialGravity()
        {
            return Vector3D.Zero;
        }

        /// <summary>Total gravity (natural + artificial). With no artificial gravity generators
        /// this equals <see cref="GetNaturalGravity"/>.</summary>
        public Vector3D GetTotalGravity()
        {
            return GetNaturalGravity();
        }

        /// <summary>The basic ship speed (magnitude of the world linear velocity).</summary>
        public double GetShipSpeed()
        {
            return Grid.Body.LinearVelocity.Length();
        }

        /// <summary>The ship's linear and angular velocity, both in the WORLD frame — returned
        /// straight from the body with no body-frame transform (source:
        /// MyShipController.cs:3233-3238). Consumers needing body-frame convert ω_B =
        /// Rᵀ·ω_W.</summary>
        public MyShipVelocities GetShipVelocities()
        {
            SimRigidBodyState state = Grid.Body.GetState();
            return new MyShipVelocities(state.LinearVelocity, new Vector3D(state.AngularVelocity));
        }

        /// <summary>Ship mass from the body: total mass is the physics mass; base mass is the total
        /// minus inventory. There is no inventory multiplier in the simulator, so physical mass
        /// equals total mass.</summary>
        public MyShipMass CalculateShipMass()
        {
            float totalMass = Grid.Body.Mass;
            float inventoryMass = TotalInventoryMass();
            float baseMass = Math.Max(0f, totalMass - inventoryMass);
            return new MyShipMass(baseMass, totalMass, totalMass);
        }

        /// <summary>Sums the inventory cargo mass of every block on the grid.</summary>
        private float TotalInventoryMass()
        {
            float sum = 0f;
            IReadOnlyList<SimTerminalBlock> blocks = Grid.TerminalSystem.Blocks;
            for (int i = 0; i < blocks.Count; i++)
            {
                sum += blocks[i].Template.InventoryMass;
            }
            return sum;
        }

        // — IMyShipController members outside the implemented contract —

        /// <inheritdoc />
        public bool HasWheels => ThrowNotSupported<bool>("IMyShipController.HasWheels");

        /// <inheritdoc />
        public bool ControlWheels
        {
            get { return ThrowNotSupported<bool>("IMyShipController.ControlWheels"); }
            set { ThrowSetNotSupported("IMyShipController.ControlWheels"); }
        }

        /// <inheritdoc />
        public bool HandBrake
        {
            get { return ThrowNotSupported<bool>("IMyShipController.HandBrake"); }
            set { ThrowSetNotSupported("IMyShipController.HandBrake"); }
        }

        /// <inheritdoc />
        public bool ShowHorizonIndicator
        {
            get { return ThrowNotSupported<bool>("IMyShipController.ShowHorizonIndicator"); }
            set { ThrowSetNotSupported("IMyShipController.ShowHorizonIndicator"); }
        }

        /// <inheritdoc />
        public bool TryGetPlanetPosition(out Vector3D position)
        {
            // No planet model exists in the simulator; planet queries are
            // outside this stage's contract.
            position = Vector3D.Zero;
            throw NotSupported("IMyShipController.TryGetPlanetPosition()");
        }

        /// <inheritdoc />
        public bool TryGetPlanetElevation(MyPlanetElevation detail, out double elevation)
        {
            // No planet model exists in the simulator; planet queries are
            // outside this stage's contract.
            elevation = 0.0;
            throw NotSupported("IMyShipController.TryGetPlanetElevation()");
        }
    }
}
