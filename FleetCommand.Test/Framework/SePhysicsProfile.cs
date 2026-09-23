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

        /// <summary>Connector capture: max separation (m) between connector points; effective
        /// threshold is the min of this and each connector's reach.</summary>
        public float ConnectorReach { get; }

        /// <summary>Connector capture: max opposing-axis angular error (rad) of the connect
        /// axes.</summary>
        public float ConnectorAngularErrorMax { get; }

        /// <summary>Connector capture: max relative linear velocity (m/s) at the connector
        /// points.</summary>
        public float ConnectorLinearVelocityMax { get; }

        /// <summary>Connector capture: max relative angular velocity (rad/s) between the connector
        /// bodies.</summary>
        public float ConnectorAngularVelocityMax { get; }

        /// <summary>Connector approach: separation (m) below which the final fixed lock (Connected)
        /// is created; set above resting contact separation.</summary>
        public float ConnectorApproachGap { get; }

        /// <summary>Connector approach: opposing-axis angular error (rad) below which the final
        /// fixed lock is created.</summary>
        public float ConnectorApproachAngle { get; }

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
            float connectorReach = 2.5f,
            float connectorAngularErrorMax = 0.174533f,
            float connectorLinearVelocityMax = 0.5f,
            float connectorAngularVelocityMax = 0.5f,
            float connectorApproachGap = 0.2f,
            float connectorApproachAngle = 0.02f,
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
            ConnectorReach = connectorReach;
            ConnectorAngularErrorMax = connectorAngularErrorMax;
            ConnectorLinearVelocityMax = connectorLinearVelocityMax;
            ConnectorAngularVelocityMax = connectorAngularVelocityMax;
            ConnectorApproachGap = connectorApproachGap;
            ConnectorApproachAngle = connectorApproachAngle;
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
