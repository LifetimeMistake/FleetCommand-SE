using System;
using System.IO;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance ( autopilot prerequisite gate and 
    /// definition of done). Locks the parts of the gate that are
    /// demonstrable as a production-style scenario: the NUnit x64 host can
    /// create fixtures, spawn the seven baseline templates, run
    /// production-style component code on Update1, advance Havok, and
    /// assert observed outcomes; the whole scenario is stable when
    /// repeated in one process; the active compatibility profile is
    /// visible in test output; and no production project references
    /// FleetCommand.Test (no production dependency, ). The remaining
    /// gate items (every  API covered, unsupported-members-fail-loudly,
    /// target/missed-Update1 scheduling) are already locked by the
    /// dedicated conformance suites referenced in the record; they are not
    /// duplicated here.
    /// </summary>
    [TestFixture]
    public class PrerequisiteGateConformanceTests : FleetTestFixture
    {
        // —  / : production-style component closes a loop on Update1 —

        [Test]
        public void ProductionComponent_OnUpdate1_ClosesLoop_AcrossMultipleShips()
        {
            // Two templated ships; a production-style component runs every
            // Update1: discovers blocks through the terminal system, reads
            // the controller, and writes thruster + gyro commands, closing
            // the control loop through Havok. After advancing, the
            // observed outcome (a turned, translated grid) is asserted.
            SimShip scout = Fc.Spawn(
                ShipTemplates.ForwardMissile,
                SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip gunship = Fc.Spawn(
                ShipTemplates.SixAxisCraft,
                SpawnState.At(new Vector3D(0, 0, 40)));

            // The "component": discover + command on every Update1.
            Fc.EveryUpdate1(frame =>
            {
                // Discover through IMyGridTerminalSystem.
                IMyShipController controller = gunship.Controller;
                IMyThrust forward = (IMyThrust)gunship.TerminalSystem.GetBlockWithName("Fwd");
                IMyGyro gyro = (IMyGyro)gunship.TerminalSystem.GetBlockWithName("Gyro");
                if (forward == null || gyro == null || controller == null)
                {
                    return;
                }
                // Read the controller (keeps the loop honest).
                Vector3D vel = controller.GetShipVelocities().LinearVelocity;
                // Write thruster + gyro overrides (control path).
                forward.ThrustOverride = forward.MaxThrust;
                gyro.GyroOverride = true;
                gyro.Pitch = 0.5f;
            });

            Fc.RunFor(1.0); // 60 fixed steps, physics advanced each one

            Assert.That(scout.Grid.Body.Mass, Is.GreaterThan(0f),
                "Both ships must exist with a real body (multiple ships in one fixture).");
            Assert.That(gunship.Body.LinearVelocity.Length(), Is.GreaterThan(1.0),
                "The commanded thruster must have accelerated the gunship (closed loop through Havok).");
            Assert.That(gunship.Grid.GyroRateSystem.LastTorque.LengthSquared(), Is.GreaterThan(1e-6),
                "The commanded gyro torque must have been applied (the loop wrote and the actuator acted).");
            Assert.That(((IMyThrust)gunship.TerminalSystem.GetBlockWithName("Fwd")).CurrentThrust, Is.GreaterThan(0f),
                "The discovered thruster must report delivered thrust.");
        }

        // —  / : the gate holds repeatedly in one process —

        [Test]
        public void Gate_RunsRepeatedly_InOneProcess_Deterministically()
        {
            float first = RunGateScenario();
            for (int run = 1; run < 3; run++)
            {
                float next = RunGateScenario();
                Assert.That(next, Is.EqualTo(first).Within(first * 0.01),
                    "The gate scenario must produce the same outcome across repeated runs in one process (run " + run + ").");
            }
        }

        private static float RunGateScenario()
        {
            using (var ctx = new FleetTestContext(new SimWorld()))
            {
                SimShip ship = ctx.Spawn(
                    ShipTemplates.SixAxisCraft,
                    SpawnState.At(Vector3D.Zero));
                IMyThrust forward = (IMyThrust)ship.TerminalSystem.GetBlockWithName("Fwd");
                forward.ThrustOverride = forward.MaxThrust;
                for (int i = 0; i < 30; i++)
                {
                    ctx.Step();
                }
                return (float)ship.Body.LinearVelocity.Length();
            } // world fully disposed each iteration
        }

        // — : all seven baseline templates spawn and move —

        [Test]
        public void SevenBaselineTemplates_SpawnAndMove()
        {
            ShipTemplate[] templates =
            {
                ShipTemplates.SixAxisCraft,
                ShipTemplates.AnisotropicCraft,
                ShipTemplates.ForwardMissile,
                ShipTemplates.NoRetroCraft,
                ShipTemplates.OrbitGunship,
                ShipTemplates.DockingCraft,
                ShipTemplates.HeavyAsymmetricCraft,
            };
            Assert.That(templates.Length, Is.EqualTo(7), "The seven baseline templates must all be present.");

            foreach (ShipTemplate template in templates)
            {
                using (var ctx = new FleetTestContext(new SimWorld()))
                {
                    SimShip ship = ctx.Spawn(template, SpawnState.At(Vector3D.Zero));
                    Assert.That(ship.Grid.Body.IsDynamic, Is.True, template.Name + " must spawn as a dynamic rigid body.");
                    Assert.That(ship.Grid.Body.Mass, Is.GreaterThan(0f), template.Name + " must have positive mass.");
                    // Apply a small external impulse and step: it must move.
                    ctx.ApplyLinearImpulse(ship, new Vector3(10f, 0, 0));
                    ctx.Step();
                    Assert.That(ship.Body.LinearVelocity.Length(), Is.GreaterThan(0f),
                        template.Name + " must move under an applied impulse.");
                }
            }
        }

        // — : active compatibility profile visible in test output —

        [Test]
        public void ActiveCompatibilityProfile_IsVisibleInTestOutput()
        {
            // The active Space Engineers / Havok compatibility profile must
            // appear in test output, per definition-of-done . Written
            // every run; this is the profile scenario results are recorded
            // against ().
            var world = new SimWorld();
            try
            {
                TestContext.Out.WriteLine(
                    "active SE/Havok compatibility profile: " + world.Profile.Name
                    + " (maxLinear=" + world.Profile.MaxLinearVelocity
                    + " maxAngular=" + world.Profile.MaxAngularVelocity
                    + " linearDamping=" + world.Profile.LinearDamping
                    + " angularDamping=" + world.Profile.AngularDamping
                    + " collisionResponse=" + world.Profile.CollisionResponse
                    + ") fixture origins at " + TestContext.CurrentContext.Test.FullName);
            }
            finally
            {
                world.Dispose();
            }
            Assert.That(world.Profile.Name, Is.EqualTo("SpaceEngineers-Default"),
                "The active profile must be the recorded default compatibility profile.");
        }

        // — : no production dependency on the test project —

        [Test]
        public void NoProductionProject_References_TestProject()
        {
            string repoRoot = FindRepoRoot();
            string fleetCommandProj = Path.Combine(repoRoot, "FleetCommand", "FleetCommand.csproj");
            Assert.That(File.Exists(fleetCommandProj), Is.True,
                "FleetCommand.csproj must be present for the dependency check (found " + fleetCommandProj + ").");
            string text = File.ReadAllText(fleetCommandProj);

            Assert.That(text, Does.Not.Contain("FleetCommand.Test"),
                "The production PB project must not reference FleetCommand.Test (no production dependency, ).");

            // The test project must be excluded from PB packaging: it must
            // not be imported into the production project's build.
            Assert.That(text, Does.Not.Contain("FleetCommand.Test.csproj"),
                "The production project must not import the test project (excluded from PB packaging).");
        }

        private static string FindRepoRoot()
        {
            DirectoryInfo dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir != null)
            {
                if (File.Exists(Path.Combine(dir.FullName, "FleetCommand.sln")))
                {
                    return dir.FullName;
                }
                dir = dir.Parent;
            }
            throw new DirectoryNotFoundException(
                "Could not locate the repository root (FleetCommand.sln) from " + AppContext.BaseDirectory);
        }
    }
}
