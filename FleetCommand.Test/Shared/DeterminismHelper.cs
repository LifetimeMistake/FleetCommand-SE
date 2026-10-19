using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;

namespace FleetCommand.Test.Shared
{
    /// <summary>
    /// Boilerplate for the determinism tests: run the same scenario once
    /// per fresh, independent world and return both outcomes so the test
    /// can assert they are equal (no wall-clock, no noise). This removes
    /// the repeated "new SimWorld, new FleetTestContext, dispose" setup
    /// that the per-suite determinism checks used to duplicate.
    /// </summary>
    public static class DeterminismHelper
    {
        /// <summary>Runs the scenario twice, once per fresh world, returning both
        /// outcomes.</summary>
        public static (T First, T Second) RunTwice<T>(Func<FleetTestContext, T> scenario)
        {
            T first = RunOnce(scenario);
            T second = RunOnce(scenario);
            return (first, second);
        }

        /// <summary>Runs the scenario once in a fresh world, disposing it afterwards.</summary>
        public static T RunOnce<T>(Func<FleetTestContext, T> scenario)
        {
            using (var context = new FleetTestContext(new SimWorld()))
            {
                return scenario(context);
            }
        }
    }
}
