using System;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Identifies and configures which Space Engineers physics behaviour the
    /// framework is reproducing. The profile carries the per-grid physical
    /// knobs that in the real game come from the live sector's environment
    /// definition (MyGridPhysics reads <c>EnvironmentDefinition.*MaxSpeed</c>
    /// for caps, and <c>MyGridPhysics</c> applies grid damping from the
    /// profile's body settings). The framework expresses these as
    /// profile-configurable values so tests can be explicit about the
    /// compatibility guarantees they rely on, rather than hardcoding a live
    /// sector lookup.
    /// </summary>
    public sealed class SePhysicsProfile
    {
        /// <summary>The display name of the active compatibility profile.</summary>
        public string Name { get; }

        /// <summary>Maximum linear velocity (m/s) applied to each dynamic rigid body.</summary>
        public float MaxLinearVelocity { get; }

        /// <summary>Maximum angular velocity (rad/s) applied to each dynamic rigid body.</summary>
        public float MaxAngularVelocity { get; }

        /// <summary>
        /// Linear damping for dynamic bodies. Zero by default; only a
        /// profile that specifies it applies non-zero damping (: "zero
        /// damping unless the selected physics profile specifies it").
        /// </summary>
        public float LinearDamping { get; }

        /// <summary>
        /// Angular damping for dynamic bodies. Zero by default.
        /// </summary>
        public float AngularDamping { get; }

        /// <summary>Whether rigid-body collision response is enabled by default.</summary>
        public bool CollisionResponse { get; }

        /// <summary>
        /// Connector docking capture: maximum separation (m) between the two
        /// connector points for capture. The effective threshold for a pair
        /// is the minimum of the two connectors' <see cref="SpaceEngineers.SimShipConnector"/>
        /// per-block reach and this value (). Part of the world
        /// physics profile and visible to tests.
        /// </summary>
        public float ConnectorReach { get; }

        /// <summary>
        /// Connector docking capture: maximum opposing-axis angular error
        /// (rad) — how far the two connectors' connect axes may deviate from
        /// exactly opposing — for capture ().
        /// </summary>
        public float ConnectorAngularErrorMax { get; }

        /// <summary>
        /// Connector docking capture: maximum magnitude (m/s) of the
        /// relative linear velocity at the two connector points for capture
        /// ().
        /// </summary>
        public float ConnectorLinearVelocityMax { get; }

        /// <summary>
        /// Connector docking capture: maximum magnitude (rad/s) of the
        /// relative angular velocity between the two connector bodies for
        /// capture ().
        /// </summary>
        public float ConnectorAngularVelocityMax { get; }

        /// <summary>
        /// Connector approach (.5): the separation (m) between the
        /// two connector points below which the approach has converged and
        /// the final fixed constraint ("lock", Connected) is created — the
        /// connectors have physically met. Set above the rigid-body
        /// contact separation the collision sim settles at, so a pair in
        /// resting face-to-face contact (as against the hulls) locks.
        /// </summary>
        public float ConnectorApproachGap { get; }

        /// <summary>
        /// Connector approach (.5): the opposing-axis angular
        /// error (rad) below which the approach has converged and the
        /// final fixed constraint is created — the connect axes are
        /// anti-parallel.
        /// </summary>
        public float ConnectorApproachAngle { get; }

        /// <summary>
        /// Remote-control autopilot (.6): the distance (m) to the
        /// current waypoint below which it is considered reached and the
        /// autopilot advances to the next one (or stops on the last) —
        /// the sim's waypoint advance radius.
        /// </summary>
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

        /// <summary>
        /// The initial, always-available profile: representative default
        /// speed caps, zero damping, collision response enabled. The real
        /// game's caps come from the live sector environment definition; this
        /// profile carries representative per-grid values test scenarios can
        /// rely on, changed through a named profile when a different
        /// guarantee is required ().
        /// </summary>
        public static readonly SePhysicsProfile Default = new SePhysicsProfile(
            "SpaceEngineers-Default",
            maxLinearVelocity: 100f,
            maxAngularVelocity: 10f,
            linearDamping: 0f,
            angularDamping: 0f,
            collisionResponse: true);
    }
}
