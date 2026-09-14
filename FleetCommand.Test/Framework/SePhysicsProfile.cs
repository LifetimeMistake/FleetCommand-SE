using System;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Identifies which Space Engineers physics behaviour the framework is
    /// reproducing. Profiles are fleshed out in a later stage; this minimal
    /// placeholder records the name so it can be surfaced in test output, and
    /// every host uses the default profile.
    /// </summary>
    public sealed class SePhysicsProfile
    {
        /// <summary>The display name of the active compatibility profile.</summary>
        public string Name { get; }

        public SePhysicsProfile(string name)
        {
            if (string.IsNullOrEmpty(name))
            {
                throw new ArgumentException("Profile name must not be empty.", nameof(name));
            }
            Name = name;
        }

        /// <summary>The initial, always-available profile.</summary>
        public static readonly SePhysicsProfile Default = new SePhysicsProfile("SpaceEngineers-Default");
    }
}
