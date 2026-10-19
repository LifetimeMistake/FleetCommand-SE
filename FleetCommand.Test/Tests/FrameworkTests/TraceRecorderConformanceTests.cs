using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Observations;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of TraceRecorder: bounded (fixed or caller-specified capacity, oldest
    /// dropped) and opt-in — nothing is recorded by default.</summary>
    [TestFixture]
    public class TraceRecorderConformanceTests : FleetTestFixture
    {
        [Test]
        public void NothingRecordedByDefault()
        {
            Fc.Step(10);
            Assert.That(Fc.World.RecorderCount, Is.EqualTo(0),
                "No time-series recorder is active unless Record is requested.");
        }

        [Test]
        public void Record_WithFixedCapacity_DropsOldest_WhenFull()
        {
            TraceRecorder<long> recorder = Fc.Record(c => c.Tick, 5);

            Fc.Step(10);

            Assert.That(recorder.Capacity, Is.EqualTo(5));
            Assert.That(recorder.Count, Is.EqualTo(5), "The recorder never exceeds its capacity.");
            Assert.That(recorder.IsFull, Is.True);
            Assert.That(recorder.ToArray(), Is.EqualTo(new long[] { 5, 6, 7, 8, 9 }),
                "When full the oldest samples are dropped (ring behaviour).");
            Assert.That(recorder.Last(), Is.EqualTo(9));
        }

        [Test]
        public void Record_WithCallerCapacity_IsRespected()
        {
            TraceRecorder<long> recorder = Fc.Record(c => c.Tick, 3);
            Fc.Step(7);
            Assert.That(recorder.Count, Is.EqualTo(3), "A caller-specified capacity is respected.");
            Assert.That(recorder.ToArray(), Is.EqualTo(new long[] { 4, 5, 6 }));
        }

        [Test]
        public void Record_RejectsNonPositiveCapacity()
        {
            Assert.Throws<ArgumentOutOfRangeException>((Action)(() => Fc.Record(c => c.Tick, 0)));
        }

        [Test]
        public void Record_SamplesFromRegistration_Only()
        {
            Fc.Step(5); // no recorder yet
            TraceRecorder<long> recorder = Fc.Record(c => c.Tick);
            Fc.Step(3);
            Assert.That(recorder.ToArray(), Is.EqualTo(new long[] { 5, 6, 7 }),
                "The recorder samples only from the tick it was requested (no backfill).");
        }

        [Test]
        public void MaximumOverLast_And_AverageOverLast_OnRecentWindow()
        {
            TraceRecorder<double> recorder = Fc.Record(c => c.Tick * 10.0);
            Fc.Step(10);

            Assert.That(recorder.MaximumOverLast(4), Is.EqualTo(90.0).Within(1e-9));
            Assert.That(recorder.MinimumOverLast(4), Is.EqualTo(60.0).Within(1e-9));
            Assert.That(recorder.AverageOverLast(4), Is.EqualTo(75.0).Within(1e-9));
        }

        [Test]
        public void EmptyRecorder_ThrowsOnLast()
        {
            TraceRecorder<long> recorder = Fc.Record(c => c.Tick, 4);
            Assert.Throws<InvalidOperationException>((Action)(() => { var _ = recorder.Last(); }));
        }
    }
}
