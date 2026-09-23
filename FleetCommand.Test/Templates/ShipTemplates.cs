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
        // Block masses (large grid, kg) are representative values so every
        // template has a positive, physically-consistent total mass.
        private const float HullMass = 2000f;
        private const float ControllerMass = 400f;
        private const float GyroMass = 100f;
        private const float ThrusterMass = 100f;
        private const float ConnectorMass = 200f;

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
            const float m = 600000f; // symmetric full-thrust authority per axis
            return ShipTemplateBuilder.LargeGrid("Six axis craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), 5000f, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, m, new Vector3I(3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Rev", Rev, 1, m, new Vector3I(-3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Up", Up, 1, m, new Vector3I(1, 3, 0), ThrusterMass)
                .AddThrusterGroup("Dwn", Dwn, 1, m, new Vector3I(1, -3, 0), ThrusterMass)
                .AddThrusterGroup("StrR", Rgt, 1, m, new Vector3I(1, 1, 3), ThrusterMass)
                .AddThrusterGroup("StrL", Lft, 1, m, new Vector3I(1, 1, -3), ThrusterMass)
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Anisotropic()
        {
            // Deliberately unequal: X 9x, Y 3x, Z 1x authority.
            const float strong = 900000f, mid = 300000f, weak = 100000f;
            return ShipTemplateBuilder.LargeGrid("Anisotropic craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), 5000f, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, strong, new Vector3I(3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Rev", Rev, 1, strong, new Vector3I(-3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Up", Up, 1, mid, new Vector3I(1, 3, 0), ThrusterMass)
                .AddThrusterGroup("Dwn", Dwn, 1, mid, new Vector3I(1, -3, 0), ThrusterMass)
                .AddThrusterGroup("StrR", Rgt, 1, weak, new Vector3I(1, 1, 3), ThrusterMass)
                .AddThrusterGroup("StrL", Lft, 1, weak, new Vector3I(1, 1, -3), ThrusterMass)
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Missile()
        {
            // Main thrust only: a strong rear cluster pushing +X and a
            // controller whose own forward axis is +X (its forward control
            // axis). No lateral or reverse thrust.
            return ShipTemplateBuilder.LargeGrid("Forward missile")
                .AddStandard("Nose", new Vector3I(0, 0, 0), 400f)
                .AddController("Pilot", new Vector3I(0, 0, 1), 200f,
                    OrientForward(new Vector3(1, 0, 0), new Vector3(0, 1, 0)))
                .AddGyro("Gyro", new Vector3I(0, -1, 0), 4000f, 60f)
                .AddThrusterGroup("Main", Fwd, 4, 700000f, new Vector3I(2, 2, 0), ThrusterMass)
                .WithSpeedLimits(150f, 15f)
                .Build();
        }

        private static ShipTemplate NoRetro()
        {
            // Authoritative forward plus full lateral (up/down/left/right);
            // explicitly NO reverse (-X) thrust.
            const float m = 500000f;
            return ShipTemplateBuilder.LargeGrid("No retro craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), 5000f, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, m, new Vector3I(3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Up", Up, 1, m, new Vector3I(1, 3, 0), ThrusterMass)
                .AddThrusterGroup("Dwn", Dwn, 1, m, new Vector3I(1, -3, 0), ThrusterMass)
                .AddThrusterGroup("StrR", Rgt, 1, m, new Vector3I(1, 1, 3), ThrusterMass)
                .AddThrusterGroup("StrL", Lft, 1, m, new Vector3I(1, 1, -3), ThrusterMass)
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Gunship()
        {
            // Anisotropic translation, generous gyro authority, and a
            // named weapon reference axis along grid +X.
            return ShipTemplateBuilder.LargeGrid("Orbit gunship")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("GyroA", new Vector3I(0, -1, 0), 8000f, GyroMass)
                .AddGyro("GyroB", new Vector3I(0, -1, 1), 8000f, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, 800000f, new Vector3I(3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Rev", Rev, 1, 250000f, new Vector3I(-3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Up", Up, 1, 400000f, new Vector3I(1, 3, 0), ThrusterMass)
                .AddThrusterGroup("Dwn", Dwn, 1, 400000f, new Vector3I(1, -3, 0), ThrusterMass)
                .AddThrusterGroup("StrR", Rgt, 1, 150000f, new Vector3I(1, 1, 3), ThrusterMass)
                .AddThrusterGroup("StrL", Lft, 1, 150000f, new Vector3I(1, 1, -3), ThrusterMass)
                .WithNamedAxis("Weapon reference", new Vector3(1, 0, 0))
                .WithSpeedLimits(120f, 12f)
                .Build();
        }

        private static ShipTemplate Docking()
        {
            // Six-axis translation plus a named front connector frame.
            const float m = 500000f;
            return ShipTemplateBuilder.LargeGrid("Docking craft")
                .AddStandard("Hull", new Vector3I(0, 0, 0), HullMass)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), 5000f, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, m, new Vector3I(3, 1, 0), ThrusterMass)
                .AddThrusterGroup("Rev", Rev, 1, m, new Vector3I(-3, 0, 0), ThrusterMass)
                .AddThrusterGroup("Up", Up, 1, m, new Vector3I(1, 3, 0), ThrusterMass)
                .AddThrusterGroup("Dwn", Dwn, 1, m, new Vector3I(1, -3, 0), ThrusterMass)
                .AddThrusterGroup("StrR", Rgt, 1, m, new Vector3I(1, 1, 3), ThrusterMass)
                .AddThrusterGroup("StrL", Lft, 1, m, new Vector3I(1, 1, -3), ThrusterMass)
                .AddConnector("Dock", new Vector3I(3, 0, 1), ConnectorMass,
                    connectorOrientation: OrientForward(new Vector3(1, 0, 0), new Vector3(0, 1, 0)))
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
            const float m = 400000f;
            return ShipTemplateBuilder.LargeGrid("Heavy asymmetric craft")
                .AddStandard("Heavy", new Vector3I(3, 0, 0), 8000f)
                .AddStandard("Center", new Vector3I(0, 0, 0), HullMass)
                .AddStandard("Light", new Vector3I(-3, 0, 0), 300f)
                .AddController("Pilot", new Vector3I(0, 0, 1), ControllerMass)
                .AddGyro("Gyro", new Vector3I(0, -1, 0), 5000f, GyroMass)
                .AddThrusterGroup("Fwd", Fwd, 1, m, new Vector3I(3, 2, 0), ThrusterMass)
                .AddThrusterGroup("Rev", Rev, 1, m, new Vector3I(-3, 2, 0), ThrusterMass)
                .AddThrusterGroup("Up", Up, 1, m, new Vector3I(1, 3, 0), ThrusterMass)
                .AddThrusterGroup("Dwn", Dwn, 1, m, new Vector3I(1, -3, 0), ThrusterMass)
                .AddThrusterGroup("StrR", Rgt, 1, m, new Vector3I(1, 1, 3), ThrusterMass)
                .AddThrusterGroup("StrL", Lft, 1, m, new Vector3I(1, 1, -3), ThrusterMass)
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
