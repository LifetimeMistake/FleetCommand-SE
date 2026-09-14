using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    /// Verifies the  clock contract (, ): time is an
    /// exact multiple of the fixed 1/60 s step, never wall-clock, and
    /// Update1/10/100 cadence is derived from the monotonic tick counter.
    /// </summary>
    [TestFixture]
    public class TickClockTests
    {
        [Test]
        public void Time_IsExactMultipleOfFixedStep()
        {
            var clock = new TickClock();
            for (int i = 1; i <= 600; i++)
            {
                clock.Advance();
                double expected = i * TickClock.FixedStep;
                Assert.That(clock.Time, Is.EqualTo(expected).Within(1e-12),
                    "Time must be an exact multiple of the fixed step at tick " + i);
            }
        }

        [Test]
        public void Tick_IsMonotonic()
        {
            var clock = new TickClock();
            long last = clock.Tick;
            for (int i = 0; i < 1000; i++)
            {
                clock.Advance();
                Assert.That(clock.Tick, Is.GreaterThan(last));
                last = clock.Tick;
            }
        }

        [Test]
        public void FixedStep_IsOneSixtiethOfASecond()
        {
            Assert.That(TickClock.FixedStep, Is.EqualTo(1.0 / 60.0).Within(1e-12));
        }

        [Test]
        public void TimeSinceLastRun_IsDerivedFromElapsedSteps()
        {
            var clock = new TickClock();
            clock.Advance(3);
            Assert.That(clock.TimeSinceLastRun(3), Is.EqualTo(3 * TickClock.FixedStep).Within(1e-12));
            Assert.That(clock.TimeSinceLastRun(10), Is.EqualTo(10 * TickClock.FixedStep).Within(1e-12));
        }

        [Test]
        public void Update10_IsDueOnTens_NotOtherwise()
        {
            var clock = new TickClock();
            for (long tick = 1; tick <= 1000; tick++)
            {
                bool expected = tick % 10 == 0;
                Assert.That(clock.IsUpdate10Due(tick), Is.EqualTo(expected),
                    "Update10 cadence at tick " + tick);
            }
        }

        [Test]
        public void Update100_IsDueOnHundreds_NotOtherwise()
        {
            var clock = new TickClock();
            for (long tick = 1; tick <= 1000; tick++)
            {
                bool expected = tick % 100 == 0;
                Assert.That(clock.IsUpdate100Due(tick), Is.EqualTo(expected),
                    "Update100 cadence at tick " + tick);
            }
        }

        [Test]
        public void UpdateCadence_IsDerivedFromTickCounter()
        {
            var clock = new TickClock();

            // Update1 is due on every tick.
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update1, 1), Is.True);
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update1, 7), Is.True);

            // Update10 on tens only.
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update10, 10), Is.True);
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update10, 11), Is.False);
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update10, 20), Is.True);

            // Update100 on hundreds only.
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update100, 100), Is.True);
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update100, 200), Is.True);
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update100, 101), Is.False);
            Assert.That(clock.IsUpdateDue(UpdateFrequency.Update100, 99), Is.False);

            // None is never due.
            Assert.That(clock.IsUpdateDue(UpdateFrequency.None, 0), Is.False);
        }
    }
}
