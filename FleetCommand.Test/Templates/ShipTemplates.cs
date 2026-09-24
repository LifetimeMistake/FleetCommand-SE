using System;
using VRageMath;

namespace FleetCommand.Test.Templates
{
    /// <summary>
    /// The framework's baseline scenarios. Each is a <see cref="ShipTemplate"/> built
    /// through the validating <see cref="ShipTemplateBuilder"/>; craft identity is purely
    /// template data, never a branch on craft type. Tests may derive variants via
    /// <see cref="ShipTemplateBuilder.From"/>. Axes: +X forward, +Y up, +Z right
    /// (force = -thruster forward, so a group with forward -X pushes +X). Every block
    /// occupies a distinct grid cell.
    /// </summary>
    public static class ShipTemplates
    {
        // Real large-grid block values (verified against the game SBCs; mass =
        // Σ component count × component mass, MyCubeBlockDefinition.cs:654-665). Hull is an
        // abstract structural block (no single game definition), kept as a representative mass.
        private const float HullMass = 2000f;
        private const float ControllerMass = 368f;    // LargeBlockCockpit (CubeBlocks_Control.sbc)
        private const float GyroMass = 12817f;        // LargeBlockGyro
        private const float GyroForce = 3.36e7f;      // LargeBlockGyro ForceMagnitude (N·m)
        private const float ConnectorMass = 3564f;     // Connector (CubeBlocks_Logistics.sbc)
        // Large ion thruster: LargeBlockLargeThrust (CubeBlocks_Thrusters.sbc) — 3x2x4, 43200
        // kg, ForceMagnitude 4.32e6 N.
        private static readonly Vector3I IonSize = new Vector3I(3, 2, 4);
        private const float IonMass = 43200f;
        private const float IonThrust = 4.32e6f;
        // Small ion thruster: LargeBlockSmallThrust — 1x1x2, 4380 kg, 3.456e5 N.
        private static readonly Vector3I SmallIonSize = new Vector3I(1, 1, 2);
        private const float SmallIonMass = 4380f;
        private const float SmallIonThrust = 3.456e5f;

        private static readonly Vector3I Fwd = new Vector3I(-1, 0, 0);  // pushes +X
        private static readonly Vector3I Rev = new Vector3I(1, 0, 0);   // pushes -X
        private static readonly Vector3I Up = new Vector3I(0, -1, 0);   // pushes +Y
        private static readonly Vector3I Dwn = new Vector3I(0, 1, 0);   // pushes -Y
        private static readonly Vector3I Rgt = new Vector3I(0, 0, -1);  // pushes +Z
        private static readonly Vector3I Lft = new Vector3I(0, 0, 1);   // pushes -Z

        /// <summary>Symmetric translation and ordinary gyros.</summary>
        public static ShipTemplate SixAxisCraft => SixAxis();
        /// <summary>Six directions with deliberately unequal authority.</summary>
        public static ShipTemplate AnisotropicCraft => Anisotropic();
        /// <summary>Main thrust only, with a forward-oriented controller axis.</summary>
        public static ShipTemplate ForwardMissile => Missile();
        /// <summary>Six-axis maneuvering authority without reverse thrust.</summary>
        public static ShipTemplate NoRetroCraft => NoRetro();
        /// <summary>Anisotropic translation, gyros, and a named weapon reference axis.</summary>
        public static ShipTemplate OrbitGunship => Gunship();
        /// <summary>Six-axis translation with a named connector frame.</summary>
        public static ShipTemplate DockingCraft => Docking();
        /// <summary>Offset mass distribution and non-isotropic inertia.</summary>
        public static ShipTemplate HeavyAsymmetricCraft => HeavyAsymmetric();

        private static ShipTemplate SixAxis()
        {
            // Symmetric full-thrust authority per axis: one real large ion thruster (4.32e6 N)
            // per direction.
            return ShipTemplateBuilder.LargeGrid("Six axis craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), GyroForce, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, IonThrust, new Vector3I(3, 0, 0), IonMass, IonSize)
                .AddThrusterGroup("Rev", Rev, 1, IonThrust, new Vector3I(-3, 0, 0), IonMass, IonSize)
                .AddThrusterGroup("Up", Up, 1, IonThrust, new Vector3I(1, 3, 0), IonMass, IonSize)
                .AddThrusterGroup("Dwn", Dwn, 1, IonThrust, new Vector3I(1, -3, 0), IonMass, IonSize)
                .AddThrusterGroup("StrR", Rgt, 1, IonThrust, new Vector3I(1, 1, 3), IonMass, IonSize)
                .AddThrusterGroup("StrL", Lft, 1, IonThrust, new Vector3I(1, 1, -3), IonMass, IonSize)
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Anisotropic()
        {
            // Deliberately unequal authority using real thruster types: X (large ion, 4.32e6 N)
            // is far stronger than Y and Z (each a small ion, 3.456e5 N).
            return ShipTemplateBuilder.LargeGrid("Anisotropic craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), GyroForce, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, IonThrust, new Vector3I(3, 0, 0), IonMass, IonSize)
                .AddThrusterGroup("Rev", Rev, 1, IonThrust, new Vector3I(-3, 0, 0), IonMass, IonSize)
                .AddThrusterGroup("Up", Up, 1, SmallIonThrust, new Vector3I(1, 3, 0), SmallIonMass, SmallIonSize)
                .AddThrusterGroup("Dwn", Dwn, 1, SmallIonThrust, new Vector3I(1, -3, 0), SmallIonMass, SmallIonSize)
                .AddThrusterGroup("StrR", Rgt, 1, SmallIonThrust, new Vector3I(1, 1, 3), SmallIonMass, SmallIonSize)
                .AddThrusterGroup("StrL", Lft, 1, SmallIonThrust, new Vector3I(1, 1, -3), SmallIonMass, SmallIonSize)
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Missile()
        {
            // Main thrust only: a strong real-ion cluster pushing +X and a
            // controller whose own forward axis is +X (its forward control
            // axis). No lateral or reverse thrust.
            return ShipTemplateBuilder.LargeGrid("Forward missile")
                .AddStandard("Nose", new Vector3I(0, 0, 0), 400f)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass,
                    OrientForward(new Vector3(1, 0, 0), new Vector3(0, 1, 0)))
                .AddGyro("Gyro", new Vector3I(0, -1, 0), GyroForce, GyroMass)
                .AddThrusterGroup("Main", Fwd, 4, IonThrust, new Vector3I(2, 2, 0), IonMass, IonSize)
                .WithSpeedLimits(150f, 15f)
                .Build();
        }

        private static ShipTemplate NoRetro()
        {
            // Authoritative forward plus full lateral (up/down/left/right);
            // explicitly NO reverse (-X) thrust.
            return ShipTemplateBuilder.LargeGrid("No retro craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), GyroForce, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, IonThrust, new Vector3I(3, 0, 0), IonMass, IonSize)
                .AddThrusterGroup("Up", Up, 1, IonThrust, new Vector3I(1, 3, 0), IonMass, IonSize)
                .AddThrusterGroup("Dwn", Dwn, 1, IonThrust, new Vector3I(1, -3, 0), IonMass, IonSize)
                .AddThrusterGroup("StrR", Rgt, 1, IonThrust, new Vector3I(1, 1, 3), IonMass, IonSize)
                .AddThrusterGroup("StrL", Lft, 1, IonThrust, new Vector3I(1, 1, -3), IonMass, IonSize)
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Gunship()
        {
            // Real thrust authority, generous double-gyro authority, and a
            // named weapon reference axis along grid +X.
            return ShipTemplateBuilder.LargeGrid("Orbit gunship")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("GyroA", new Vector3I(0, -1, 0), GyroForce, GyroMass)
                .AddGyro("GyroB", new Vector3I(0, -1, 1), GyroForce, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, IonThrust, new Vector3I(3, 0, 0), IonMass, IonSize)
                .AddThrusterGroup("Rev", Rev, 1, SmallIonThrust, new Vector3I(-3, 0, 0), SmallIonMass, SmallIonSize)
                .AddThrusterGroup("Up", Up, 1, SmallIonThrust, new Vector3I(1, 3, 0), SmallIonMass, SmallIonSize)
                .AddThrusterGroup("Dwn", Dwn, 1, SmallIonThrust, new Vector3I(1, -3, 0), SmallIonMass, SmallIonSize)
                .AddThrusterGroup("StrR", Rgt, 1, SmallIonThrust, new Vector3I(1, 1, 3), SmallIonMass, SmallIonSize)
                .AddThrusterGroup("StrL", Lft, 1, SmallIonThrust, new Vector3I(1, 1, -3), SmallIonMass, SmallIonSize)
                .WithNamedAxis("Weapon reference", new Vector3(1, 0, 0))
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Docking()
        {
            // Six-axis translation plus a front connector facing +X (grid-external), selected by
            // ConnectDirection (-1,0,0) → attach axis a = CD.X·Left = +X.
            return ShipTemplateBuilder.LargeGrid("Docking craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), GyroForce, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, IonThrust, new Vector3I(3, 1, 0), IonMass, IonSize)
                .AddThrusterGroup("Rev", Rev, 1, IonThrust, new Vector3I(-3, 0, 0), IonMass, IonSize)
                .AddThrusterGroup("Up", Up, 1, IonThrust, new Vector3I(1, 3, 0), IonMass, IonSize)
                .AddThrusterGroup("Dwn", Dwn, 1, IonThrust, new Vector3I(1, -3, 0), IonMass, IonSize)
                .AddThrusterGroup("StrR", Rgt, 1, IonThrust, new Vector3I(1, 1, 3), IonMass, IonSize)
                .AddThrusterGroup("StrL", Lft, 1, IonThrust, new Vector3I(1, 1, -3), IonMass, IonSize)
                .AddConnector("Dock", new Vector3I(3, 0, 1), ConnectorMass,
                    connectDirection: new Vector3(-1, 0, 0))
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate HeavyAsymmetric()
        {
            // A heavy 8000 kg block offset to +X and a light 300 kg tail at
            // -X give an offset center of mass and a deliberately
            // non-isotropic inertia tensor (the parallel-axis term dominates
            // along the offset axis), while retaining full translation
            // authority.
            return ShipTemplateBuilder.LargeGrid("Heavy asymmetric craft")
                .AddStandard("Heavy", new Vector3I(3, 0, 0), 8000f)
                .AddStandard("Center", new Vector3I(0, 0, 0), HullMass)
                .AddStandard("Light", new Vector3I(-3, 0, 0), 300f)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), GyroForce, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, IonThrust, new Vector3I(3, 2, 0), IonMass, IonSize)
                .AddThrusterGroup("Rev", Rev, 1, IonThrust, new Vector3I(-3, 2, 0), IonMass, IonSize)
                .AddThrusterGroup("Up", Up, 1, IonThrust, new Vector3I(1, 3, 0), IonMass, IonSize)
                .AddThrusterGroup("Dwn", Dwn, 1, IonThrust, new Vector3I(1, -3, 0), IonMass, IonSize)
                .AddThrusterGroup("StrR", Rgt, 1, IonThrust, new Vector3I(1, 1, 3), IonMass, IonSize)
                .AddThrusterGroup("StrL", Lft, 1, IonThrust, new Vector3I(1, 1, -3), IonMass, IonSize)
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        /// <summary>
        /// Builds a grid-local orientation whose forward and up
        /// basis vectors are the given world directions (used for the
        /// controller's forward axis and connector's connect face).
        /// </summary>
        private static Quaternion OrientForward(Vector3 forward, Vector3 up)
        {
            Matrix basis = Matrix.Identity;
            basis.Forward = Vector3.Normalize(forward);
            Vector3 right = Vector3.Normalize(Vector3.Cross(up, forward));
            basis.Up = Vector3.Normalize(Vector3.Cross(forward, right));
            basis.Right = right;
            return Quaternion.CreateFromRotationMatrix(basis);
        }
    }
}
