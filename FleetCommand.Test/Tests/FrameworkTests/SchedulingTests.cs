using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Verifies Update scheduling and callback cadence: coherent start-of-tick state, elapsed-fixed-step TimeSinceLastRun, suppression, and registration-ordered tick mutations.</summary>
    [TestFixture]
    public class SchedulingTests : FleetTestFixture
    {
        [Test]
        public void EveryUpdate1_RunsEveryTick_WithCoherentStartOfTickTime()
        {
            var ticksSeen = new List<long>();
            var timesSeen = new List<double>();

            Fc.EveryUpdate1(frame =>
            {
                ticksSeen.Add(Fc.Tick);
                timesSeen.Add(frame.Time);
            });

            Fc.Step(10);

            Assert.That(ticksSeen.Count, Is.EqualTo(10), "An Update1 callback runs on every tick.");
            for (int i = 0; i < 10; i++)
            {
                Assert.That(ticksSeen[i], Is.EqualTo(i),
                    "Callback sees the start-of-tick tick, not the post-step value.");
                Assert.That(timesSeen[i], Is.EqualTo(i * TickClock.FixedStep).Within(1e-12),
                    "Callback sees start-of-tick simulation time.");
            }
        }

        [Test]
        public void EveryUpdate1_TimeSinceLastRun_AccumulatesOverMissedUpdate1()
        {
            var since = new List<double>();
            Fc.EveryUpdate1(frame => since.Add(frame.TimeSinceLastRun));

            Fc.Step();               // tick 0, callback runs, since = 0
            Fc.World.SuppressNextUpdate1();
            Fc.Step();               // tick 1, callback suppressed
            Fc.Step();               // tick 2, callback runs, since = 2 steps

            Assert.That(since.Count, Is.EqualTo(2));
            Assert.That(since[0], Is.EqualTo(0).Within(1e-12));
            Assert.That(since[1], Is.EqualTo(2 * TickClock.FixedStep).Within(1e-12),
                "TimeSinceLastRun must carry the interval accumulated across the missed Update1.");
        }

        [Test]
        public void SuppressedUpdate1_DoesNotSkipPhysics()
        {
            int callbacks = 0;
            Fc.EveryUpdate1(frame => callbacks++);

            Fc.Step();   // callback runs
            Fc.Step();   // callback runs
            Fc.World.SuppressNextUpdate1();
            Fc.Step();   // callback suppressed (missed update)
            Fc.Step();   // callback runs again

            // One callback was suppressed; physics still advanced all four ticks.
            Assert.That(callbacks, Is.EqualTo(3));
            Assert.That(Fc.Tick, Is.EqualTo(4), "Suppressing an Update1 must not skip physics.");
        }

        [Test]
        public void Suppression_IsOneShot()
        {
            int callbacks = 0;
            Fc.EveryUpdate1(frame => callbacks++);

            Fc.World.SuppressNextUpdate1();
            Fc.Step();   // suppressed
            Fc.Step();   // runs
            Fc.Step();   // runs

            Assert.That(callbacks, Is.EqualTo(2), "Suppression applies to a single tick only.");
        }

        [Test]
        public void AtTick_Mutation_SeesStartOfTickTime_OnTargetTick()
        {
            long seenTick = -1;
            double seenTime = -1;
            Fc.AtTick(5, c => { seenTick = c.Tick; seenTime = c.Time; });

            Fc.Step(6);

            Assert.That(seenTick, Is.EqualTo(5), "Mutation ran on its target tick.");
            Assert.That(seenTime, Is.EqualTo(5 * TickClock.FixedStep).Within(1e-12),
                "Mutation sees start-of-tick time of its target tick.");
        }

        [Test]
        public void AtTick_MutatingScheduler_FromWithinMutation_IsSupported()
        {
            var order = new List<int>();
            Fc.AtTick(1, c =>
            {
                order.Add(1);
                c.AtTick(1, cc => order.Add(0)); // appended to same tick's list
            });
            Fc.AtTick(1, c => order.Add(2));

            Fc.Step(2);

            Assert.That(order, Is.EqualTo(new List<int> { 1, 2, 0 }),
                "A mutation may schedule more work for the same tick; it runs after already-registered actions.");
        }

        [Test]
        public void RunUntil_StopsOnFirstSatisfyingTick_ReturnsTrue()
        {
            bool result = Fc.RunUntil(c => c.Tick >= 7, 5.0);

            Assert.That(result, Is.True);
            Assert.That(Fc.Tick, Is.EqualTo(7),
                "RunUntil stops at the first tick whose completed-state observation satisfies the predicate.");
        }

        [Test]
        public void RunUntil_ReturnsFalse_OnTimeout()
        {
            bool result = Fc.RunUntil(c => false, 0.5);
            Assert.That(result, Is.False);
        }

        [Test]
        public void RunFor_StepsFullDuration_EvenForPartialInterval()
        {
            Fc.RunFor(1.0);
            Assert.That(Fc.Tick, Is.EqualTo(60), "Running for one second advances 60 fixed steps.");

            Fc.RunFor(1.0 / 120.0);
            Assert.That(Fc.Tick, Is.EqualTo(61), "A partial step is covered by rounding up.");
        }
    }
}
