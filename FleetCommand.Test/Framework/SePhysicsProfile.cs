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

        public SePhysicsProfile(
            string name,
            float maxLinearVelocity = 0f,
            float maxAngularVelocity = 0f,
            float linearDamping = 0f,
            float angularDamping = 0f,
            bool collisionResponse = true)
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
