using System;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Identifies and configures which SE physics behaviour the framework reproduces: the
    /// per-grid knobs that in the game come from the live sector env definition
    /// (MyGridPhysics reads <c>EnvironmentDefinition.*MaxSpeed</c>), expressed as
    /// profile-configurable values so tests are explicit about the guarantees they rely on.
    /// </summary>
    public sealed class SePhysicsProfile
    {
        /// <summary>The display name of the active compatibility profile.</summary>
        public string Name { get; }

        /// <summary>Maximum linear velocity (m/s) applied to each dynamic rigid body.</summary>
        public float MaxLinearVelocity { get; }

        /// <summary>Maximum angular velocity (rad/s) applied to LARGE-grid dynamic bodies
        /// (2.5 m cells), the game's large-ship angular-rate cap
        /// (MyGridPhysics.LargeShipMaxAngularVelocityLimit, MyGridPhysics.cs:158).</summary>
        public float MaxAngularVelocityLarge { get; }

        /// <summary>Maximum angular velocity (rad/s) applied to SMALL-grid dynamic bodies
        /// (0.5 m cells), the game's small-ship angular-rate cap
        /// (MyGridPhysics.SmallShipMaxAngularVelocityLimit, MyGridPhysics.cs:159).</summary>
        public float MaxAngularVelocitySmall { get; }

        /// <summary>Linear damping for dynamic bodies; zero unless a profile specifies
        /// it.</summary>
        public float LinearDamping { get; }

        /// <summary>Angular damping for dynamic bodies; the game's default is
        /// MyPerGameSettings.DefaultAngularDamping = 0.1 (MyPerGameSettings.cs:137,
        /// applied in MyGridPhysics.cs:436).</summary>
        public float AngularDamping { get; }

        /// <summary>Whether rigid-body collision response is enabled by default.</summary>
        public bool CollisionResponse { get; }

        /// <summary>Connector auto-attach gate: the squared distance (m^2) between the two
        /// connection positions must be below this for TryAttach to create the approach
        /// constraint. The game's literal value (MyShipConnector.cs:1486) —
        /// <c>(vector3D2 - vector3D).LengthSquared() &lt; 0.3499999940395355</c> — so the two
        /// faces must be within ≈0.59 m.</summary>
        public float ConnectorAttachSeparationSquared { get; }

        /// <summary>Connector approach-keep: if the squared distance between the two connection
        /// positions rises above this while approaching (not yet welded), the pair is detached.
        /// The game's literal value (MyShipConnector.cs:1574) — LengthSquared &gt; 0.5 — so the
        /// pair must stay within ≈0.7 m.</summary>
        public float ConnectorKeepSeparationSquared { get; }

        /// <summary>Connector auto-attach orientation gate: the angle between the two connect
        /// directions (in world) must exceed this for the faces to be considered opposed. The
        /// game's literal value (MyShipConnector.cs:1488, :1492) — <c>3π/4</c> (135°), i.e. the
        /// connect directions opposed within 45°.</summary>
        public float ConnectorOpposingAngle { get; }

        /// <summary>Remote-control autopilot: distance (m) to the current waypoint below which it
        /// is considered reached (waypoint advance radius).</summary>
        public float AutopilotArrivalDistance { get; }

        public SePhysicsProfile(
            string name,
            float maxLinearVelocity = 0f,
            float maxAngularVelocityLarge = 0f,
            float maxAngularVelocitySmall = 0f,
            float linearDamping = 0f,
            float angularDamping = 0f,
            bool collisionResponse = true,
            float connectorAttachSeparationSquared = 0.35f,
            float connectorKeepSeparationSquared = 0.5f,
            float connectorOpposingAngle = 2.356194490192345f,
            float autopilotArrivalDistance = 3f)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("Profile name must not be empty.", nameof(name));
            }
            Name = name;
            MaxLinearVelocity = maxLinearVelocity;
            MaxAngularVelocityLarge = maxAngularVelocityLarge;
            MaxAngularVelocitySmall = maxAngularVelocitySmall;
            LinearDamping = linearDamping;
            AngularDamping = angularDamping;
            CollisionResponse = collisionResponse;
            ConnectorAttachSeparationSquared = connectorAttachSeparationSquared;
            ConnectorKeepSeparationSquared = connectorKeepSeparationSquared;
            ConnectorOpposingAngle = connectorOpposingAngle;
            AutopilotArrivalDistance = autopilotArrivalDistance;
        }

        /// <summary>The game's large-ship angular-rate limit (MyGridPhysics.cs:158),
        /// ToRadians(18000) ≈ 314.159 rad/s; the env default takes the same value
        /// (MyEnvironmentDefinition.cs:108-114) so the effective cap is this.</summary>
        public static readonly float LargeShipMaxAngularSpeed = 100f * (float)Math.PI;

        /// <summary>The game's small-ship angular-rate limit (MyGridPhysics.cs:159),
        /// ToRadians(36000) ≈ 628.319 rad/s; the env default takes the same value
        /// (MyEnvironmentDefinition.cs:108-114) so the effective cap is this.</summary>
        public static readonly float SmallShipMaxAngularSpeed = 200f * (float)Math.PI;

        /// <summary>The initial, always-available profile: the game's per-grid-size angular
        /// rate caps (effectively non-binding under gyro command), the game's 0.1 angular
        /// damping, collision enabled.</summary>
        public static readonly SePhysicsProfile Default = new SePhysicsProfile(
            "SpaceEngineers-Default",
            maxLinearVelocity: 100f,
            maxAngularVelocityLarge: LargeShipMaxAngularSpeed,
            maxAngularVelocitySmall: SmallShipMaxAngularSpeed,
            linearDamping: 0f,
            angularDamping: 0.1f,
            collisionResponse: true);
    }
}
