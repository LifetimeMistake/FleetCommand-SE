using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Validation.Tests
{
    public class SettlingGateTests
    {
        static SettleDecision Rest(SettlingGate gate)
        {
            return gate.Observe(Vector3D.Zero, QuaternionD.Identity, Vector3D.Zero,
                Vector3D.Zero, Vector3D.Zero, 1.0 / 60);
        }

        [Test]
        public void Rest_Requires15BrakeFrames_Then60PassiveFrames()
        {
            var gate = new SettlingGate();
            for (int i = 0; i < 14; i++) Assert.That(Rest(gate), Is.EqualTo(SettleDecision.Brake));
            Assert.That(Rest(gate), Is.EqualTo(SettleDecision.BeginCoast));
            for (int i = 0; i < 59; i++) Assert.That(Rest(gate), Is.EqualTo(SettleDecision.Coast));
            Assert.That(Rest(gate), Is.EqualTo(SettleDecision.Ready));
        }

        [Test]
        public void PositionMotion_RejectsStaleZeroVelocityReadings()
        {
            var gate = new SettlingGate();
            Rest(gate);
            for (int i = 1; i <= 90; i++)
                Assert.That(gate.Observe(new Vector3D(i * 0.001, 0, 0), QuaternionD.Identity,
                    Vector3D.Zero, Vector3D.Zero, Vector3D.Zero, 1.0 / 60), Is.EqualTo(SettleDecision.Brake));
            Assert.That(gate.ObservedSpeed, Is.EqualTo(0.06).Within(1e-6));
        }

        [Test]
        public void CockpitVelocity_DisagreementPreventsSettlement()
        {
            var gate = new SettlingGate();
            for (int i = 0; i < 90; i++)
                Assert.That(gate.Observe(Vector3D.Zero, QuaternionD.Identity, Vector3D.Zero,
                    Vector3D.UnitX, Vector3D.Zero, 1.0 / 60), Is.EqualTo(SettleDecision.Brake));
        }

        [Test]
        public void MotionDuringPassiveCoast_RestartsBraking()
        {
            var gate = new SettlingGate();
            for (int i = 0; i < 15; i++) Rest(gate);
            Assert.That(gate.Observe(Vector3D.UnitX, QuaternionD.Identity, Vector3D.Zero,
                Vector3D.Zero, Vector3D.Zero, 1.0 / 60), Is.EqualTo(SettleDecision.Brake));
            Assert.That(gate.StableTicks, Is.Zero);
            Assert.That(gate.PassiveTicks, Is.Zero);
        }

        [Test]
        public void OrientationMotion_RejectsStaleZeroAngularVelocity()
        {
            var gate = new SettlingGate();
            Rest(gate);
            for (int i = 1; i < 90; i++)
                Assert.That(gate.Observe(Vector3D.Zero, QuaternionD.CreateFromAxisAngle(Vector3D.Up, i * 0.002),
                    Vector3D.Zero, Vector3D.Zero, Vector3D.Zero, 1.0 / 60), Is.EqualTo(SettleDecision.Brake));
        }
    }
}
