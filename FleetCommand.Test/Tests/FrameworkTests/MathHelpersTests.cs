using FleetCommand.Common;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    public class MathHelpersTests
    {
        [TestCase(1f, float.Epsilon, 1f)]
        [TestCase(-1f, float.Epsilon, 1f)]
        [TestCase(1f, -float.Epsilon, 1f)]
        [TestCase(1f, 0.0000000001f, 10000000000f)]
        public void AiThrustRatio_ReplacesOverflowButPreservesFiniteQuotients(float numerator, float denominator, float expected)
        {
            Assert.That(MathHelpers.SafeRatio(numerator, denominator), Is.EqualTo(expected));
        }
    }
}
