using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of the failure ring buffer: it fills on an uncaught component exception, carries recent actuator history, and is bounded.</summary>
    [TestFixture]
    public class FailureRingBufferConformanceTests : FleetTestFixture
    {
        [Test]
        public void UncaughtComponentException_FillsFailureRing_WithExceptionAndRecentHistory()
        {
            SimShip ship = Fc.Spawn(HullRig(1000f), SpawnState.At(Vector3D.Zero));
            int calls = 0;
            Fc.EveryUpdate1(frame =>
            {
                calls++;
                if (calls == 3)
                {
                    throw new InvalidOperationException("boom in component");
                }
            });

            Assert.Throws<InvalidOperationException>((Action)(() =>
            {
                Fc.Step(); // tick 0
                Fc.Step(); // tick 1
                Fc.Step(); // tick 2 -> throws
            }));

            // The ring buffer has recent actuator history (from the ticks
            // that completed) plus the exception entry.
            Assert.That(Fc.World.FailureLog.IsEmpty, Is.False, "The failure ring buffer is populated.");
            string[] lines = Fc.World.FailureLog.ToArray();
            bool hasActuatorHistory = false;
            bool hasException = false;
            for (int i = 0; i < lines.Length; i++)
            {
                if (lines[i].Contains("grid " + ship.Id))
                {
                    hasActuatorHistory = true; // a recent actuator snapshot line
                }
                if (lines[i].Contains("uncaught component exception")
                    && lines[i].Contains("InvalidOperationException")
                    && lines[i].Contains("boom in component"))
                {
                    hasException = true;
                }
            }
            Assert.That(hasActuatorHistory, Is.True, "Recent actuator history is retained in the failure ring buffer.");
            Assert.That(hasException, Is.True, "The uncaught component exception is recorded in the failure ring buffer.");
        }

        [Test]
        public void FailureRingBuffer_IsBounded()
        {
            SimShip ship = Fc.Spawn(HullRig(1000f), SpawnState.At(Vector3D.Zero));
            Fc.Step(200);

            Assert.That(Fc.World.FailureLog.Capacity, Is.EqualTo(24));
            Assert.That(Fc.World.FailureLog.Count, Is.LessThanOrEqualTo(Fc.World.FailureLog.Capacity),
                "The failure ring buffer never grows beyond its short, fixed capacity.");
        }

        private static ShipTemplate HullRig(float hullMass)
        {
            return new ShipTemplate("Hull", 0.5f, new[]
            {
                new TemplateBlock("Hull", new Vector3I(0, 0, 0), hullMass, 0.5f),
            });
        }
    }
}
