using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Sbc;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Validation.Launcher;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Validation.Tests
{
    public class StationProbeTests
    {
        [Test]
        public void DockMission_InspectingEitherLcdDoesNotAbortAndPublishesAfterClosing()
        {
            string flight = null;
            string report = SimulationRun.Execute(SimulationRun.StationBlueprint, Vector3D.Zero, MatrixD.Identity,
                command: "dockprobe", additionalArtifact: (kind, text) => flight = text,
                afterStart: (context, ship, runner) => {
                    var dockLcd = (SimTextPanel)ship.TerminalSystem.GetBlockWithName("FC Dock Report");
                    var flightLcd = (SimTextPanel)ship.TerminalSystem.GetBlockWithName("FC Report");
                    context.Step(180);
                    dockLcd.IsTextPanelOpen = true; flightLcd.IsTextPanelOpen = true;
                    context.Step(180);
                    Assert.That(runner.IsRunning, Is.True, runner.Report);
                    int ticks = 0;
                    while (runner.IsRunning && ticks++ < 20000) context.Step();
                    Assert.That(runner.Report, Does.Contain("END|COMPLETE|9|"));
                    dockLcd.IsTextPanelOpen = false; flightLcd.IsTextPanelOpen = false;
                    context.Step(30);
                    Assert.That(((SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation")).Runtime.UpdateFrequency,
                        Is.EqualTo(UpdateFrequency.None));
                });
            Assert.That(report, Does.Contain("END|COMPLETE|9|"));
            Assert.That(flight, Does.Contain("END|COMPLETE|6|"));
        }

        [Test]
        public void DockMission_ReloadDuringRetreatStopsThePersistentActuators()
        {
            var file = new SbcFileReader().ReadFile(SimulationRun.StationBlueprint);
            var ids = new HashSet<long>();
            foreach (var grid in file.Grids)
            {
                Assert.That(ids.Add(grid.SerializedEntityId.Value), Is.True, "Each serialized entity ID must be unique for game remapping.");
                foreach (var block in grid.Blocks)
                    Assert.That(ids.Add((long)block.Data.Element("EntityId")), Is.True,
                        "Duplicate blueprint entity ID: " + (string)block.Data.Element("EntityId"));
            }
            using (var context = new FleetTestContext(new SimWorld()))
            {
                var ship = AssemblySpawn.Spawn(context, file, Vector3D.Zero, MatrixD.Identity);
                context.Step(2);
                var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                var host = new ValidationHost(ship.TerminalSystem, pb.Runtime, pb, _ => { }, "");
                pb.RunHandler = arg => host.Main(arg, pb.CurrentUpdateSource);
                pb.Run("dockprobe", UpdateType.Terminal); context.Step(180);
                Assert.That(host.Report, Does.Contain("PHASE|retreat|"));
                var restored = new ValidationHost(ship.TerminalSystem, pb.Runtime, pb, _ => { }, host.Report);
                Assert.That(restored.IsRunning, Is.False);
                Assert.That(restored.Report, Does.Contain("END|INTERRUPTED|RELOAD"));
                foreach (var block in ship.Grid.TerminalSystem.Blocks)
                {
                    var thrust = block as IMyThrust;
                    if (thrust != null) Assert.That(thrust.ThrustOverridePercentage, Is.Zero);
                    var gyro = block as IMyGyro;
                    if (gyro != null) Assert.That(gyro.GyroOverride, Is.False);
                }
                Assert.That(pb.Runtime.UpdateFrequency, Is.EqualTo(UpdateFrequency.None));
                Assert.That(ship.Controller.DampenersOverride, Is.True);
            }
        }

        [Test]
        public void DockMission_LostCaptureRestartsTheContinuousObservationBeforeLock()
        {
            string report = SimulationRun.Execute(SimulationRun.StationBlueprint, Vector3D.Zero, MatrixD.Identity,
                command: "dockprobe", afterStart: (context, ship, runner) => {
                    var port = (IMyShipConnector)ship.TerminalSystem.GetBlockWithName("FC Connector");
                    int waited = 0;
                    while (!runner.Report.Contains("CHECK|capture_without_lock|PASS") && runner.IsRunning && waited++ < 20000) context.Step();
                    Assert.That(port.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
                    port.Enabled = false; context.Step(80);
                    Assert.That(port.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
                    port.Enabled = true;
                    int reacquired = -1, locked = -1;
                    for (int i = 0; i < 600 && runner.IsRunning; i++)
                    {
                        context.Step();
                        if (port.Status == MyShipConnectorStatus.Connectable && reacquired < 0) reacquired = i;
                        if (port.Status == MyShipConnectorStatus.Connected) { locked = i; break; }
                    }
                    Assert.That(reacquired, Is.GreaterThanOrEqualTo(0));
                    Assert.That(locked - reacquired, Is.GreaterThanOrEqualTo(59),
                        "Lock must wait for a new continuous 60-frame capture after loss.");
                    runner.Main("abort", UpdateType.Terminal);
                });
            Assert.That(report, Does.Contain("END|ABORTED|OPERATOR|"));
        }

        [Test]
        public void AssemblyFixture_PreservesStaticStationAndInitiallyConnectedCraft()
        {
            var file = new SbcFileReader().ReadFile(SimulationRun.StationBlueprint);
            using (var context = new FleetTestContext(new SimWorld()))
            {
                var ship = AssemblySpawn.Spawn(context, file, new Vector3D(200, -300, 400),
                    MatrixD.CreateFromYawPitchRoll(0.3, -0.2, 0.4));
                var connector = (IMyShipConnector)ship.TerminalSystem.GetBlockWithName("FC Connector");
                Assert.That(file.Grids.Count, Is.EqualTo(2));
                Assert.That(context.World.Grids.Count, Is.EqualTo(2));
                Assert.That(ship.Grid.IsStatic, Is.False);
                Assert.That(connector.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
                Assert.That(connector.OtherConnector.CubeGrid.IsStatic, Is.True);
                context.Step(60);
                Assert.That(connector.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
                Assert.That(ship.Grid.Body.LinearVelocity.Length(), Is.LessThan(0.001));
            }
        }

        [TestCase(0, 0, 0)]
        [TestCase(0.7, -0.3, 1.2)]
        public void DockMission_ExportsCompleteDockingAndFlightReports_AndEndsConnected(double yaw, double pitch, double roll)
        {
            string flight = null;
            bool running = false;
            var report = SimulationRun.Execute(SimulationRun.StationBlueprint, new Vector3D(2500, -300, 800), MatrixD.CreateFromYawPitchRoll(yaw, pitch, roll),
                command: "dockprobe", additionalArtifact: (kind, text) => flight = text,
                afterStart: (context, ship, runner) => running = runner.IsRunning);
            Assert.That(running, Is.True, report);
            Assert.That(report, Does.Contain("END|COMPLETE|9|"), report);
            Assert.That(report, Does.Contain("CHECK|capture_without_lock|PASS"));
            Assert.That(report, Does.Contain("CHECK|explicit_lock|PASS"));
            Assert.That(report.Length, Is.LessThan(ValidationReport.Limit));
            Assert.That(flight, Does.Contain("END|COMPLETE|"));
            Assert.That(flight.Length, Is.LessThan(ValidationReport.Limit));
        }

        [Test]
        public void DockMission_AbortClearsActuatorsAndPreservesTheReport()
        {
            string report = SimulationRun.Execute(SimulationRun.StationBlueprint, Vector3D.Zero, MatrixD.Identity,
                command: "dockprobe", afterStart: (context, ship, runner) => {
                    context.Step(180);
                    runner.Main("abort", UpdateType.Terminal);
                    foreach (var block in ship.Grid.TerminalSystem.Blocks)
                    {
                        var thrust = block as IMyThrust;
                        if (thrust != null) Assert.That(thrust.ThrustOverridePercentage, Is.Zero);
                        var gyro = block as IMyGyro;
                        if (gyro != null) Assert.That(gyro.GyroOverride, Is.False);
                    }
                    Assert.That(ship.Controller.DampenersOverride, Is.True);
                });
            Assert.That(report, Does.Contain("END|ABORTED|OPERATOR|"));
        }

        [Test]
        public void DockMission_RequiresInitialConnectionAndReportsTheError()
        {
            string report = SimulationRun.Execute(SimulationRun.StationBlueprint, Vector3D.Zero, MatrixD.Identity,
                command: "dockprobe", beforeStart: (context, ship) =>
                    ((IMyShipConnector)ship.TerminalSystem.GetBlockWithName("FC Connector")).Disconnect());
            Assert.That(report, Does.Contain("END|ERROR|START_CONNECTED|"));
        }
    }
}
