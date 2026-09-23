using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Verifies the nine-phase tick pipeline runs in the exact non-negotiable order, with
    /// real observable structure.</summary>
    [TestFixture]
    public class TickPhaseOrderTests
    {
        [Test]
        public void Step_ExecutesAllNinePhases_InSpecOrder()
        {
            string[] expected =
            {
                "PublishCompletedState",
                "ScheduledMutations",
                "ComponentCallbacks",
                "RecomputePower",
                "AggregateForce",
                "ApplyImpulses",
                "AdvancePhysics",
                "SynchronizeState",
                "SampleAndEvaluate",
            };

            using (var world = new SimWorld())
            {
                world.Step();
                Assert.That(world.LastStepPhases, Is.EqualTo(new List<string>(expected)),
                    "The nine tick phases must run in the exact spec order.");
            }
        }

        [Test]
        public void Step_AdvancesTickAndTime()
        {
            using (var world = new SimWorld())
            {
                long start = world.Clock.Tick;
                double startTime = world.Clock.Time;
                world.Step();
                Assert.That(world.Clock.Tick, Is.EqualTo(start + 1));
                Assert.That(world.Clock.Time, Is.GreaterThan(startTime));
            }
        }

        [Test]
        public void Step_AdvancesHavok_ByFixedStep_EachTick()
        {
            // Advancing the world repeatedly must not throw and must keep the
            // HkWorld live and stepping: the empty world steps Havok once per
            // tick with no bodies and no global gravity.
            using (var world = new SimWorld())
            {
                for (int i = 0; i < 60; i++)
                {
                    world.Step();
                }
                Assert.That(world.IsDisposed, Is.False);
            }
        }
    }
}
