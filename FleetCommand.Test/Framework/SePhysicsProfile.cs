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

        /// <summary>Maximum angular velocity (rad/s) applied to each dynamic rigid body.</summary>
        public float MaxAngularVelocity { get; }

        /// <summary>Linear damping for dynamic bodies; zero unless a profile specifies
        /// it.</summary>
        public float LinearDamping { get; }

        /// <summary>Angular damping for dynamic bodies; zero by default.</summary>
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
            float maxAngularVelocity = 0f,
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
            MaxAngularVelocity = maxAngularVelocity;
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

        /// <summary>The initial, always-available profile: representative default speed caps, zero
        /// damping, collision enabled.</summary>
        public static readonly SePhysicsProfile Default = new SePhysicsProfile(
            "SpaceEngineers-Default",
            maxLinearVelocity: 100f,
            maxAngularVelocity: 10f,
            linearDamping: 0f,
            angularDamping: 0f,
            collisionResponse: true);
    }
}
