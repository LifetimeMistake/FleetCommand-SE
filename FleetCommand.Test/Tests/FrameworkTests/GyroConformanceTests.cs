using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Shared;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of the gyro block and grid gyro-rate system: channel signs, override
    /// ramp, saturation, angular-rate cap, and power scaling.</summary>
    [TestFixture]
    public class GyroConformanceTests : FleetTestFixture
    {
        // — gyro channel signs (all-three-negated, MyGyro.cs:141-190) —

        [Test]
        public void Gyro_ChannelSigns_AllThreeNegated_LocalOverrideVelocity()
        {
            // Identity-mounted gyro: GyroOverrideVelocityGrid == local
            // override velocity. Each ingame setter must store the
            // negation on its axis: Pitch -> X, Yaw -> Y, Roll -> Z.
            SimShip ship = Fc.Spawn(
                HullPlusGyro("id", hullMass: 3000f, gyroForce: 1e6f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            SimGyro sim = (SimGyro)g;
            g.GyroOverride = true;

            g.Pitch = 3f;
            AssertVector3(sim.GyroOverrideVelocityGrid, new Vector3(-3f, 0, 0), 1e-4,
                "Pitch setter must store -value on the local X axis (all-three-negated).");
            Assert.That(g.Pitch, Is.EqualTo(3f).Within(1e-4), "Pitch getter must return the ingame value.");

            g.Yaw = 2f;
            AssertVector3(sim.GyroOverrideVelocityGrid, new Vector3(-3f, -2f, 0), 1e-4,
                "Yaw setter must store -value on the local Y axis.");

            g.Roll = 1f;
            AssertVector3(sim.GyroOverrideVelocityGrid, new Vector3(-3f, -2f, -1f), 1e-4,
                "Roll setter must store -value on the local Z axis.");

            Assert.That(sim.GyroOverrideVelocityGrid, Is.EqualTo(new Vector3(-3f, -2f, -1f)),
                "m_gyroOverrideVelocity = (-Pitch, -Yaw, -Roll) for an identity mount.");
        }

        [Test]
        public void Gyro_ChannelSigns_GettersReflectNegatedOverride_AndResetByZero()
        {
            SimShip ship = Fc.Spawn(
                HullPlusGyro("s", hullMass: 3000f, gyroForce: 1e6f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Yaw = 4f;
            Assert.That(g.Yaw, Is.EqualTo(4f).Within(1e-4));
            g.Yaw = -4f; // setter again negates
            SimGyro sim = (SimGyro)g;
            AssertVector3(sim.GyroOverrideVelocityGrid, new Vector3(0, 4f, 0), 1e-4,
                "Yaw=-4 must store +4 on local Y (X=-Pitch, Y=-Yaw, Z=-Roll).");
            g.Yaw = 0f;
            AssertVector3(sim.GyroOverrideVelocityGrid, Vector3.Zero, 1e-4,
                "Clearing the channel must clear the override velocity.");
        }

        [Test]
        public void Gyro_ChannelSigns_SetOnlyWhenOverrideEnabled()
        {
            // The game's setters only take effect while GyroOverride is
            // true (MyGyro.cs:149-154 etc.); writing before enabling must
            // be ignored.
            SimShip ship = Fc.Spawn(
                HullPlusGyro("gate", hullMass: 3000f, gyroForce: 1e6f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            SimGyro sim = (SimGyro)g;

            g.GyroOverride = false;
            g.Pitch = 2f;
            AssertVector3(sim.GyroOverrideVelocityGrid, Vector3.Zero, 1e-6,
                "A channel write while override is disabled must be ignored.");

            g.GyroOverride = true;
            g.Pitch = 2f;
            AssertVector3(sim.GyroOverrideVelocityGrid, new Vector3(-2f, 0, 0), 1e-6,
                "With override enabled the channel write must apply.");
        }

        [Test]
        public void Gyro_SixOrientations_TransformLocalOverrideToGrid()
        {
            // Remount the gyro so its local +X points along each of the six
            // grid axes; a pure Pitch command (local override (-p,0,0))
            // must come out as -p along that grid axis. Locks the channel
            // signs end-to-end against GyroOverrideVelocityGrid for
            // differently mounted gyros.
            float pitch = 2.5f;
            foreach (Vector3 axis in new[] { Vector3.Right, Vector3.Left, Vector3.Up, Vector3.Down, Vector3.Forward, Vector3.Backward })
            {
                string name = "Gx" + Math.Abs(axis.X) + Math.Abs(axis.Y) + Math.Abs(axis.Z);
                SimShip ship = Fc.Spawn(
                    HullPlusGyro(name, hullMass: 1000f, gyroForce: 1e6f,
                        orientation: RotateLocalXTo(axis)),
                    SpawnState.At(Vector3D.Zero));
                IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
                SimGyro sim = (SimGyro)g;
                g.GyroOverride = true;
                g.Pitch = pitch;

                // Predicted from the channel rule ((-p,0,0) local) and the
                // mounting geometry: -p along the mounted grid axis.
                Vector3 expected = Vector3.TransformNormal(new Vector3(-pitch, 0, 0),
                    Matrix.CreateFromQuaternion(RotateLocalXTo(axis)));
                AssertVector3(sim.GyroOverrideVelocityGrid, expected, 1e-3,
                    "A Pitch command on a gyro mounted with local X->" + axis + " must give the predicted grid-space override.");
            }
        }

        [Test]
        public void Gyro_Power_And_ForceScaling()
        {
            SimShip ship = Fc.Spawn(
                HullPlusGyro("p", hullMass: 3000f, gyroForce: 1e6f, gyroPowerDemand: 8f),
                SpawnState.At(Vector3D.Zero));
            SimGyro sim = (SimGyro)ship.TerminalSystem.GetBlockWithName("G");
            IMyGyro g = sim;

            Assert.That(sim.MaxGyroForce, Is.EqualTo(1e6f).Within(1e-2), "MaxGyroForce at full power is the template force.");
            Assert.That(sim.RequiredPowerInput, Is.EqualTo(8f).Within(1e-4));

            g.GyroPower = 0.5f;
            Assert.That(sim.GyroPower, Is.EqualTo(0.5f).Within(1e-4));
            Assert.That(sim.MaxGyroForce, Is.EqualTo(5e5f).Within(1e-2),
                "MaxGyroForce must scale with GyroPower (force x power).");
            Assert.That(sim.RequiredPowerInput, Is.EqualTo(4f).Within(1e-4),
                "RequiredPowerInput must scale with GyroPower.");
            Assert.That(sim.PowerDemand, Is.EqualTo(4f).Within(1e-4),
                "PowerDemand (fed to the bus) is the power-scaled required input.");

            g.GyroPower = 5f;
            Assert.That(sim.GyroPower, Is.EqualTo(1f).Within(1e-4), "GyroPower must clamp to 1.");
            g.GyroPower = -1f;
            Assert.That(sim.GyroPower, Is.EqualTo(0f).Within(1e-4), "GyroPower must clamp to 0.");
        }

        // — override acceleration ramp (analytic, MyGridGyroSystem.cs:306-324) —

        [Test]
        public void OverrideRamp_RampFrameCounts_AreTheGamePolicy()
        {
            // Large error (|diff|^2 > 2.467401) -> 120 frames; otherwise
            // (int)(|diff|^2 * 48.228886) + 1.
            Assert.That(GyroRateSystem.RampFrameSqThreshold, Is.EqualTo(2.467401f).Within(1e-6),
                "The large-rate threshold must match the game source (MyGridGyroSystem.cs:312).");
            Assert.That(GyroRateSystem.ComputeOverrideRampFrames(new Vector3(3f, 0, 0)), Is.EqualTo(120),
                "A large rate error must use the 120-frame ramp.");
            Assert.That(GyroRateSystem.ComputeOverrideRampFrames(new Vector3(2f, 0, 0)), Is.EqualTo(120),
                "|diff|^2 = 4 (> 2.467401) is a large-rate ramp.");

            // |diff|^2 just under the threshold: small-rate formula.
            Vector3 small = new Vector3(1f, 0, 0); // |diff|^2 = 1
            int expected = (int)(1f * 48.228886f) + 1; // 49
            Assert.That(GyroRateSystem.ComputeOverrideRampFrames(small), Is.EqualTo(expected),
                "A small rate error must use (int)(|diff|^2 * 48.228886) + 1 frames.");

            Vector3 tiny = new Vector3(0.05f, 0, 0); // |diff|^2 ~ 0.0025 -> 1 frame
            Assert.That(GyroRateSystem.ComputeOverrideRampFrames(tiny), Is.EqualTo(1),
                "A tiny error must floor at 1 ramp frame.");
        }

        [Test]
        public void OverrideRamp_FirstStepAppliesErrorDividedByRampFrames()
        {
            // From rest with a small command that does not saturate, the
            // very first step's grid-frame angular velocity change is
            // diff / rampFrames (the acceleration-ramp policy), much less
            // than the target: the controller ramps, it does not apply the
            // full target instantly.
            float hullMass = 1500f;
            SimShip ship = Fc.Spawn(
                HullPlusGyro("ramp", hullMass: hullMass, gyroForce: 4e6f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Pitch = 1.4f; // |target|^2 = 1.96 < 2.467401 -> small-rate ramp

            Fc.Step(); // 1 step
            SimRigidBody body = ship.Grid.Body;

            // error = target - 0 = (-1.4, 0, 0); rampFrames = (int)(1.96 * 48.228886) + 1.
            int frames = (int)(1.4f * 1.4f * 48.228886f) + 1;
            float expected = 1.4f / frames; // |diff| / rampFrames, grid frame (~ -X)
            float actual = Math.Abs(body.AngularVelocity.X);
            Assert.That(actual, Is.GreaterThan(0));
            Assert.That(actual, Is.EqualTo(expected).Within(expected * 0.5),
                "The first override step must ramp: ~ |diff| / rampFrames of the target, not the full target.");
            Assert.That(actual, Is.LessThan(1.4f * 0.5f),
                "After one step the grid must still be well short of the target (the ramp is not an instant step).");
        }

        [Test]
        public void OverrideRamp_CountsDownAcrossTicks_ReArmsOnOverrideChange()
        {
            // The game's override ramp is stateful: armed once (on a new
            // gyro-set / override change), it counts down across ticks, and
            // is re-armed only when the gyro set changes
            // (MyGridGyroSystem.cs:306-324 — RecomputeGyroParameters on
            // m_gyrosChanged resets m_overrideAccelerationRampFrames; the
            // per-tick decrement is the live countdown). Lock the
            // cross-tick countdown and the re-arm-on-change, so a stable
            // override ramps its proportional gain up over time rather than
            // re-deriving 120 frames from the (shrinking) error every tick.
            SimShip ship = Fc.Spawn(
                HullPlusGyro("rampct", hullMass: 12000f, gyroForce: 2e6f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            GyroRateSystem sys = ship.Grid.GyroRateSystem;
            g.GyroOverride = true;
            g.Pitch = 6f; // |target|^2 = 36 > 2.467401 -> 120-frame (large) ramp

            Fc.Step(); // arming step (Aggregate sees the gyro-set change)
            Assert.That(sys.OverrideRampFrames, Is.EqualTo(120),
                "A large rate error must arm the 120-frame ramp.");

            Fc.Step(); // no config change
            Assert.That(sys.OverrideRampFrames, Is.EqualTo(119),
                "With a stable override the ramp must count down across ticks, not re-arm to 120 each tick.");

            Fc.Step();
            Assert.That(sys.OverrideRampFrames, Is.EqualTo(118),
                "The cross-tick countdown must continue while the override is stable.");

            // Changing the override re-arms the ramp (the game's
            // SetGyroTorque -> MarkDirty -> RecomputeGyroParameters resets
            // the ramp so the new command ramps from scratch).
            g.Yaw = 3f; // marks the gyro set dirty
            Fc.Step();
            Assert.That(sys.OverrideRampFrames, Is.EqualTo(120),
                "Changing the override must re-arm the 120-frame ramp (not continue the old countdown).");
        }

        // — override drives rotation in the all-three-negated direction (end-to-end) —

        [Test]
        public void Override_PositivePitch_SpinsGrid_NegativeGridX()
        {
            // Identity gyro, positive Pitch -> local override (-p,0,0) ->
            // grid target -X. The grid must rotate about -grid X (the
            // all-three-negated ingame sign, end-to-end through Havok).
            SimShip ship = Fc.Spawn(
                HullPlusGyro("dir", hullMass: 4000f, gyroForce: 4e6f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Pitch = 2f;

            Fc.RunFor(4.0);

            Assert.That(ship.Grid.Body.AngularVelocity.X, Is.LessThan(-1f),
                "Positive Pitch must spin the grid about -grid X (all-three-negated mapping, end-to-end).");
        }

        [Test]
        public void Override_PositiveYaw_SpinsGrid_NegativeGridY()
        {
            SimShip ship = Fc.Spawn(
                HullPlusGyro("d", hullMass: 4000f, gyroForce: 4e6f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Yaw = 2f;

            Fc.RunFor(4.0);

            Assert.That(ship.Grid.Body.AngularVelocity.Y, Is.LessThan(-1f),
                "Positive Yaw must spin the grid about -grid Y (all-three-negated, end-to-end).");
        }

        // — override saturation to the aggregate gyro-force sphere —

        [Test]
        public void Override_Saturated_WhenDemandExceedsGyroForceSphere()
        {
            // A heavy grid with one weak gyro and a large commanded rate:
            // the torque needed to reach the target exceeds the aggregate
            // gyro-force sphere, so the override clamps (saturates).
            SimShip ship = Fc.Spawn(
                HullPlusGyro("sat", hullMass: 20000f, gyroForce: 3e4f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Pitch = 9f; // near the 10 rad/s cap, needs far more torque than 3e4 N·m

            Fc.Step();

            GyroRateSystem sys = ship.Grid.GyroRateSystem;
            Assert.That(sys.LastSaturated, Is.True,
                "A torque demand beyond the aggregate gyro force must set the saturation flag.");
            // The applied torque is clamped to the gyro-force sphere radius
            // (maxOverrideForce when ControlTorque is 0).
            Assert.That(sys.LastTorque.Length(), Is.LessThanOrEqualTo(3e4f + 1e-2),
                "The applied torque must not exceed the aggregate gyro-force sphere.");
        }

        // — stopping a spinning grid —

        [Test]
        public void Override_ZeroTarget_StopsSpinningGrid()
        {
            // Gyro override with all channels at 0 (target rate 0) must
            // bring an initially rotating grid to rest (error opposes the
            // rotation and torque closes on 0).
            SimShip ship = Fc.Spawn(
                HullPlusGyro("stop", hullMass: 4000f, gyroForce: 4e6f),
                SpawnState.At(Vector3D.Zero).WithAngularVelocity(new Vector3(1, 0, 0)));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true; // all channels 0

            Fc.RunFor(2.0);

            Assert.That(ship.Grid.Body.AngularVelocity.Length(), Is.LessThan(1e-2),
                "An override target of 0 must stop the grid's rotation.");
            Assert.That(ship.Grid.Body.AngularVelocity.X, Is.GreaterThan(-0.05f),
                "Stopping must decelerate toward zero, not overshoot to a large counter-rotation.");
        }

        // — non-identity world-orientation: the override reads R^T·ω, not R·ω —

        [Test]
        public void Override_RotatedGrid_ReadsTransposeOfWorldOmega_NotOmegaItself()
        {
            // The override loop must read the body's WORLD angular
            // velocity into the GRID frame as R^T·ω (the game reads via
            // WorldMatrixInvScaled = R^T, MyGridGyroSystem.cs:281-285),
            // then convert the grid-frame corrective torque back to the
            // WORLD frame as R·(gridTorque) when applying it. Every
            // identity-orientation grid (R = I) has R == R^T, which hid
            // an accumulated mis-transpose bug where the read used R·ω
            // (R instead of R^T). This test yaws the grid to a genuine
            // non-identity WORLD orientation so R != R^T, and asserts
            // the observed grid-frame override torque against the R^T
            // analytic value — which under the bug would come out
            // sign-flipped (R²-style) and fail.
            //
            // Scenario (chosen for an exact, by-hand derivation):
            //   R = 90° about world Z (grid yawed a quarter turn).
            //   R·(x,y,z) = (-y, x, z);  R^T·(x,y,z) = (y, -x, z).
            // So R != R^T here — e.g. R^T·(1,0,0) = (0,-1,0) but
            // R·(1,0,0) = (0,1,0).
            //   ω_world = (2, 0, 0): the grid spins about world X.
            //   Override target 0 (all gyro channels at 0, like a stop).
            //
            // Grid-frame read (correct, R^T): R^T·(2,0,0) = (0,-2,0).
            // Rate error = target - read = -(0,-2,0) = (0, +2, 0) ->
            // the corrective grid torque is along +grid Y, and applied
            // to the world it becomes R·(+Y) = (-1,0,0)·k, opposing the
            // world-X spin (physically correct). Under the bug the read
            // would be R·(2,0,0) = (0,+2,0) -> error (0,-2,0) -> a
            // grid torque along -Y, which applied to the world
            // ACCELERATES the world-X spin. The sign of LastTorque.Y is
            // therefore the whole point.
            //   Ramp (MyGridGyroSystem.cs:306-324): |diff|^2 = 4 >
            // 2.467401 -> 120 frames. Torque = diff·(60/120) /
            // inverse-inertia (componentwise), far inside the
            // gyro-force sphere (radius = maxOverrideForce) so no clamp.
            // Nothing else perturbs: zero gravity, a single gyro, one
            // fixed step.
            MatrixD R = MatrixD.CreateRotationZ(MathHelper.PiOver2);
            Vector3 omegaWorld = new Vector3(2f, 0, 0);

            SimShip ship = Fc.Spawn(
                GyroCubeCraft(gyroForce: 4e6f),
                SpawnState.At(Vector3D.Zero, R).WithAngularVelocity(omegaWorld));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true; // all channels 0 -> override target rate 0

            Fc.Step(); // first override step: read ω into grid frame, torque, apply

            GyroRateSystem sys = ship.Grid.GyroRateSystem;
            SimRigidBody body = ship.Grid.Body;

            // Independent analytic expectation, derived from the game
            // formula and the chosen geometry (R^T read, ramp policy,
            // inverse-inertia scaling). The inverse-inertia diagonal is
            // the body's own (not under test); the frame read is the
            // discriminated quantity.
            Vector3 gridOmega = Vector3.Transform(omegaWorld, Matrix.Transpose((Matrix)R)); // R^T·ω = (0,-2,0)
            Vector3 diff = -gridOmega; // error = target(0) - R^T·ω = (0, +2, 0)
            Assert.That(diff.Y, Is.EqualTo(2f).Within(1e-2), "R^T·(2,0,0) must read as a -2 ~Y grid spin.");
            Assert.That(GyroRateSystem.ComputeOverrideRampFrames(diff), Is.EqualTo(120),
                "|diff|^2 = 4 is a large rate error -> the 120-frame ramp.");

            Vector3 inv = body.InverseInertiaDiagonal;
            Vector3 expected = new Vector3(
                diff.X * (60f / 120f) / inv.X,
                diff.Y * (60f / 120f) / inv.Y,
                diff.Z * (60f / 120f) / inv.Z);
            Assert.That(expected.Length(), Is.GreaterThan(1f),
                "Sanity: the corrective torque must be a substantial, measurable vector.");

            Vector3 actual = sys.LastTorque;
            AssertVector3(actual, expected, expected.Y * 1e-2 + 1e-2,
                "A 90°-yawed grid spinning about world X must register the R^T (not R) grid-frame corrective torque");

            // Lock the sign explicitly: R^T gives +Y; the mis-transpose
            // (R·ω) would give -Y, so a clearly positive Y at this
            // magnitude is the discriminating assertion.
            Assert.That(actual.Y, Is.GreaterThan(expected.Y * 0.9f),
                "The grid-frame corrective torque must be along +grid Y (R^T read), not -Y (the R read bug would give -Y).");
        }

        // — grid angular-rate cap (the body's configured MaxAngularVelocity) —

        [Test]
        public void Override_DoesNotExceed_GridAngularRateCap()
        {
            // Command the max rate; the gyro and Havok must keep the
            // grid's angular velocity within the configured cap even while
            // the override keeps pushing at full force.
            SimShip ship = Fc.Spawn(
                HullPlusGyro("cap", hullMass: 5000f, gyroForce: 4e6f, maxAngular: 4f),
                SpawnState.At(Vector3D.Zero));
            IMyGyro g = (IMyGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Pitch = 10f; // clamped to the 4 rad/s cap anyway

            Fc.RunFor(3.0);

            float cap = ship.Grid.Body.MaxAngularVelocity;
            Assert.That(cap, Is.EqualTo(4f).Within(1e-4), "The template sets the grid angular-rate cap.");
            Assert.That(Math.Abs(ship.Grid.Body.AngularVelocity.X), Is.LessThanOrEqualTo(cap + 0.05),
                "The grid's angular velocity must not exceed its configured angular-rate cap.");
        }

        // — power shortfall scales gyro torque (the bus's DeliveredRatio) —

        [Test]
        public void PowerShortfall_ScalesGyroTorque_AndResponse()
        {
            // A 10 MW gyro on a 5 MW battery delivers ratio 0.5; its
            // response must be proportionally weaker than a fully powered
            // twin.
            SimShip weak = Fc.Spawn(
                PoweredGyroCraft(maxOutput: 5f, gyroPowerDemand: 10f),
                SpawnState.At(Vector3D.Zero));
            SimShip full = Fc.Spawn(
                PoweredGyroCraft(maxOutput: 30f, gyroPowerDemand: 10f),
                SpawnState.At(new Vector3D(0, 0, 60)));
            SimGyro w = (SimGyro)weak.TerminalSystem.GetBlockWithName("G");
            SimGyro f = (SimGyro)full.TerminalSystem.GetBlockWithName("G");
            w.GyroOverride = true;
            f.GyroOverride = true;
            w.Yaw = 4f;
            f.Yaw = 4f;

            Fc.RunFor(4.0);

            Assert.That(w.DeliveredRatio, Is.EqualTo(0.5f).Within(1e-6),
                "The phase-4 bus must give the 10 MW gyro a 0.5 ratio from the 5 MW battery.");
            Assert.That(f.DeliveredRatio, Is.EqualTo(1f).Within(1e-6));
            Assert.That(Math.Abs(weak.Grid.Body.AngularVelocity.Y), Is.LessThan(Math.Abs(full.Grid.Body.AngularVelocity.Y)),
                "The power-scaled gyro must achieve a weaker response than the fully powered one.");
        }

        // — aggregation: force-weighted override target — and working-state gating —

        [Test]
        public void Override_ForceWeightedTarget_AggregatesAcrossGyros()
        {
            // Two override gyros with different forces command opposing
            // rates; the aggregate target must be the force-weighted
            // average, and an aggregate is emitted as one resultant.
            SimShip ship = Fc.Spawn(GyroPairCraft(), SpawnState.At(Vector3D.Zero));
            SimGyro a = (SimGyro)ship.TerminalSystem.GetBlockWithName("A");
            SimGyro b = (SimGyro)ship.TerminalSystem.GetBlockWithName("B");
            a.GyroOverride = true;
            b.GyroOverride = true;
            a.Yaw = 4f;      // (0,-4,0) local
            b.Yaw = -2f;     // (0,2,0) local

            Fc.Step();

            // Weighted average of grid rates: (3 * -4 + 1 * 2) / 4 = -2.5 (Y).
            GyroRateSystem sys = ship.Grid.GyroRateSystem;
            Assert.That(sys.OverrideTargetVelocity.Y, Is.EqualTo(-2.5f).Within(1e-3),
                "The aggregate target must be the force-weighted average of the override rates.");
        }

        [Test]
        public void Override_DisabledGyro_ContributesNothing()
        {
            SimShip ship = Fc.Spawn(
                HullPlusGyro("off", hullMass: 3000f, gyroForce: 1e6f),
                SpawnState.At(Vector3D.Zero).WithAngularVelocity(new Vector3(1, 0, 0)));
            SimGyro g = (SimGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Enabled = false;

            Fc.RunFor(1.0);

            // A disabled gyro is not used: no override force, so it cannot
            // act on the grid (which keeps rotating).
            Assert.That(ship.Grid.Body.AngularVelocity.Length(), Is.GreaterThan(0.8f),
                "A disabled gyro must contribute no override authority (the grid keeps rotating).");
        }

        [Test]
        public void Override_AutopilotEnabled_DivertsOverrideGyrosIntoAuthorityPool()
        {
            SimShip ship = Fc.Spawn(
                HullPlusGyro("ap", hullMass: 3000f, gyroForce: 1e6f),
                SpawnState.At(Vector3D.Zero).WithAngularVelocity(new Vector3(1, 0, 0)));
            SimGyro g = (SimGyro)ship.TerminalSystem.GetBlockWithName("G");
            g.GyroOverride = true;
            g.Pitch = 4f;
            ship.Grid.GyroRateSystem.AutopilotEnabled = true; // override -> authority pool, target stays 0

            Fc.RunFor(4.0);

            // Diverted into the authority pool: no override target, so no
            // override-loop actuation. With no override force the engine
            // routes to the manual path (MyGridGyroSystem.UpdateGyros),
            // which — with ControlTorque zero — damps the free spin toward
            // rest via its angular-velocity slowdown (:231-250, a ~1 s
            // time constant, so several seconds to a small residual).
            // This is the game's real behavior (a gyro with no control
            // stops a free rotation), so the residual spin is damped.
            Assert.That(ship.Grid.GyroRateSystem.OverrideTargetVelocity.Length(), Is.LessThan(1e-6),
                "With AutopilotEnabled the override gyro must not contribute to the override target.");
            Assert.That(ship.Grid.Body.AngularVelocity.Length(), Is.LessThan(0.1f),
                "Diverted override gyros must not drive the grid via the override loop (the manual path damps the free spin).");
        }

        // — helpers —

        private static void AssertVector3(Vector3 actual, Vector3 expected, double tolerance, string message)
        {
            Assert.Multiple((Action)(() =>
            {
                Assert.That(actual.X, Is.EqualTo(expected.X).Within(tolerance), message + " (X)");
                Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(tolerance), message + " (Y)");
                Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(tolerance), message + " (Z)");
            }));
        }

        /// <summary>A single gyro block at the grid origin (a clean symmetric cube for the analytic
        /// scenario).</summary>
        private static ShipTemplate GyroCubeCraft(float gyroForce)
        {
            return new ShipTemplate("gyro-cube", 2.5f, new[]
            {
                new TemplateBlock("G", Vector3I.Zero, 1000f, 2.5f,
                    kind: BlockKind.Gyro, gyroForce: gyroForce),
            });
        }

        /// <summary>A heavy hull plus one gyro block (name "G").</summary>
        private static ShipTemplate HullPlusGyro(string name, float hullMass, float gyroForce,
            Quaternion? orientation = null, float gyroPowerDemand = 0f, float? maxAngular = null)
        {
            var blocks = new List<TemplateBlock>
            {
                new TemplateBlock("Hull", Vector3I.Zero, hullMass, 2.5f),
                new TemplateBlock("G", new Vector3I(2, 0, 0), 400f, 2.5f,
                    kind: BlockKind.Gyro, gyroForce: gyroForce,
                    gyroPowerDemand: gyroPowerDemand, orientation: orientation),
            };
            var template = new ShipTemplate(name, 2.5f, blocks,
                maxAngularVelocity: maxAngular ?? 0f);
            return template;
        }

        /// <summary>A battery + hull + gyro ("G") with the given bus shortfall.</summary>
        private static ShipTemplate PoweredGyroCraft(float maxOutput, float gyroPowerDemand)
        {
            return CraftTemplates.PoweredDriverCraft("pg", maxOutput, 100f, new TemplateBlock(
                "G", new Vector3I(4, 0, 0), 400f, 2.5f,
                kind: BlockKind.Gyro, gyroForce: 4e6f, gyroPowerDemand: gyroPowerDemand));
        }

        /// <summary>Two gyros (A strong, B weak) for aggregation.</summary>
        private static ShipTemplate GyroPairCraft()
        {
            return new ShipTemplate("pair", 2.5f, new[]
            {
                new TemplateBlock("Hull", Vector3I.Zero, 3000f, 2.5f),
                new TemplateBlock("A", new Vector3I(2, 0, 0), 400f, 2.5f,
                    kind: BlockKind.Gyro, gyroForce: 3e6f),
                new TemplateBlock("B", new Vector3I(3, 0, 0), 400f, 2.5f,
                    kind: BlockKind.Gyro, gyroForce: 1e6f),
            });
        }

        /// <summary>An orientation that maps the block's local +X onto
        /// <paramref name="gridAxis"/>.</summary>
        private static Quaternion RotateLocalXTo(Vector3 gridAxis)
        {
            Vector3 from = Vector3.Right;
            float dot = Vector3.Dot(from, gridAxis);
            if (dot > 0.9999f)
            {
                return Quaternion.Identity;
            }
            if (dot < -0.9999f)
            {
                return Quaternion.CreateFromAxisAngle(Vector3.Up, (float)Math.PI);
            }
            Vector3 axis = Vector3.Cross(from, gridAxis);
            axis.Normalize();
            float angle = (float)Math.Acos(MathHelper.Clamp(dot, -1f, 1f));
            return Quaternion.CreateFromAxisAngle(axis, angle);
        }
    }
}
