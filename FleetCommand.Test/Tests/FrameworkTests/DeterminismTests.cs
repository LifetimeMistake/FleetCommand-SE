using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    /// Verifies the deterministic guarantee (): two runs of the same
    /// scenario on the same build step identically — same tick, time, and
    /// callback sequence. Time is derived from the fixed step, never
    /// wall-clock, so no run-to-run variation can creep in.
    /// </summary>
    [TestFixture]
    public class DeterminismTests : FleetTestFixture
    {
        [Test]
        public void TwoRunsOfSameScenario_StepIdentically()
        {
            ScenarioResult first = RunScenario();
            ScenarioResult second = RunScenario();

            Assert.That(second.StepPhaseSequences, Is.EqualTo(first.StepPhaseSequences),
                "The nine-phase sequence must be identical between the two runs.");
            Assert.That(second.CallbackTimes, Is.EqualTo(first.CallbackTimes),
                "Callback times must be identical between the two runs.");
            Assert.That(second.SinceTimes, Is.EqualTo(first.SinceTimes),
                "TimeSinceLastRun values must be identical between the two runs.");
            Assert.That(second.MutationOrder, Is.EqualTo(first.MutationOrder),
                "Scheduled mutation order must be identical between the two runs.");
            Assert.That(second.EndTick, Is.EqualTo(first.EndTick));
            Assert.That(second.EndTime, Is.EqualTo(first.EndTime).Within(1e-12));
        }

        private sealed class ScenarioResult
        {
            public readonly List<List<string>> StepPhaseSequences = new List<List<string>>();
            public readonly List<double> CallbackTimes = new List<double>();
            public readonly List<double> SinceTimes = new List<double>();
            public readonly List<int> MutationOrder = new List<int>();
            public long EndTick;
            public double EndTime;
        }

        private static ScenarioResult RunScenario()
        {
            var result = new ScenarioResult();
            using (var context = new FleetTestContext(new SimWorld()))
            {
                context.AtTick(2, c => result.MutationOrder.Add(10));
                context.AtTick(2, c => result.MutationOrder.Add(11));
                context.AtTick(5, c => result.MutationOrder.Add(20));
                context.AtTick(5, c => result.MutationOrder.Add(21));

                context.EveryUpdate1(frame =>
                {
                    result.CallbackTimes.Add(frame.Time);
                    if (frame.TimeSinceLastRun > 0)
                    {
                        result.SinceTimes.Add(frame.TimeSinceLastRun);
                    }
                });

                // Run 12 steps, capturing the phase sequence of each.
                for (int i = 0; i < 12; i++)
                {
                    context.Step();
                    result.StepPhaseSequences.Add(new List<string>(context.World.LastStepPhases));
                }

                result.EndTick = context.Tick;
                result.EndTime = context.Time;
            }
            return result;
        }
    }

    /// <summary>
    /// Verifies that scheduled mutations run in deterministic order —
    /// registration order, then stable numeric ID order () — so a
    /// test's observable mutation sequence is stable across runs.
    /// </summary>
    [TestFixture]
    public class MutationOrderingTests : FleetTestFixture
    {
        [Test]
        public void MutationsForSameTick_RunInRegistrationOrder()
        {
            var order = new List<int>();
            // Register out of numeric order to prove it is registration
            // order (not value or ID order) that governs same-tick
            // execution.
            Fc.AtTick(4, c => order.Add(2));
            Fc.AtTick(4, c => order.Add(1));
            Fc.AtTick(4, c => order.Add(3));

            Fc.Step(5);

            Assert.That(order, Is.EqualTo(new List<int> { 2, 1, 3 }),
                "Mutations registered for the same tick run in registration order.");
        }

        [Test]
        public void MutationsForDifferentTicks_RunOnTheirOwnTick_InTickOrder()
        {
            var order = new List<long>();
            Fc.AtTick(9, c => order.Add(Fc.Tick));
            Fc.AtTick(3, c => order.Add(Fc.Tick));
            Fc.AtTick(6, c => order.Add(Fc.Tick));

            Fc.Step(10);

            Assert.That(order, Is.EqualTo(new List<long> { 3, 6, 9 }),
                "Mutations for different ticks run at their target tick, in tick order.");
        }
    }
}
