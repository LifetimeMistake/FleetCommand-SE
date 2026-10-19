using System;
using NUnit.Framework;

namespace FleetCommand.Test.Shared
{
    /// <summary>
    /// Shared assertion helpers for the FleetCommand test suite. This is a
    /// test-only fixture location (not FleetCommand.Common) — nothing here
    /// ships in a program.
    /// </summary>
    public static class TestHelpers
    {
        /// <summary>
        /// Asserts that evaluating <paramref name="accessor"/> throws a
        /// <see cref="NotSupportedException"/> whose message names
        /// <paramref name="member"/>. An unimplemented interface member
        /// must throw naming the member, never silently default.
        /// </summary>
        public static void AssertThrowsNamed(Func<object> accessor, string member)
        {
            try
            {
                accessor();
            }
            catch (NotSupportedException ex)
            {
                Assert.That(ex.Message, Does.Contain(member),
                    "An unimplemented member must throw naming the member '" + member + "', not a silent default.");
                return;
            }
            Assert.Fail("Expected NotSupportedException naming '" + member + "', but the member did not throw.");
        }
    }
}
