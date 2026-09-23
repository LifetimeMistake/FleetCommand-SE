using System;
using System.Collections.Generic;
using FleetCommand.Common;
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
    /// <summary>Conformance of the connector block and the real game connector lifecycle (a
    /// port of MyShipConnector): auto-detect + attach through the malleable hinge, the
    /// approach-keep, and the weld (fixed constraint) on contact.</summary>
    [TestFixture]
    public class ConnectorConformanceTests : FleetTestFixture
    {
        // — connector point kinematics (world pose + point velocity from the body) —

        [Test]
        public void ConnectorPoint_WorldPose_FromOwnedGridBody()
        {
            SimShip ship = Fc.Spawn(
                ConnectorCraft("pose", hullMass: 1000f),
                SpawnState.At(new Vector3D(0, 0, 7)));
            SimShipConnector con = Connector(ship);

            // Block "C" is at template cell (2,0,0); small grid (0.5 m)
            // -> block centre local (1,0,0). At identity grid orientation
            // the world position is origin + (1,0,0). The connection
            // point sits at the connector's mating face: offset from the
            // block centre along the connect axis (identity forward
            // (0,0,-1)) by the block's half-extent (0.25).
            Vector3D expected = new Vector3D(0, 0, 7) + new Vector3D(1, 0, 0) + new Vector3D(0, 0, -0.25);
            AssertDot(con.WorldPosition, expected, 1e-3,
                "The connection point must be the block's world position offset to the mating face.");

            // Default connect axis (identity mount, block identity) is the
            // connection frame's local forward = (0,0,-1) in world.
            AssertDot(con.ConnectAxisWorld, new Vector3D(0, 0, -1), 1e-3,
                "An identity-mounted connector must extend along the connection frame's local forward (0,0,-1).");
        }

        [Test]
        public void ConnectorPoint_PointVelocity_FromOwnedGridBody()
        {
            SimShip ship = Fc.Spawn(
                ConnectorCraft("pv", hullMass: 1000f),
                SpawnState.At(Vector3D.Zero)
                    .WithAngularVelocity(new Vector3(0, 2f, 0)));
            SimShipConnector con = Connector(ship);
            SimRigidBody body = ship.Grid.Body;

            Vector3D comWorld = Vector3D.Transform(new Vector3D(body.CenterOfMassLocal), ship.Grid.WorldMatrix);
            Vector3D r = con.WorldPosition - comWorld;
            Vector3 omega = body.AngularVelocity;
            Vector3D expected = body.LinearVelocity + new Vector3D(Vector3.Cross(omega, (Vector3)r));

            Assert.That(mathDist(con.PointVelocityWorld, expected) < 1e-3,
                "The connection point velocity must be v + ω × r derived from the owning rigid body.");

            Assert.That(con.PointVelocityWorld.Z, Is.LessThan(-0.5),
                "A spin about Y must impart a -Z velocity at the +X-offset connection point (ω × r).");
        }

        // — stored scenario state: supported getters; setters throw naming the member —

        [Test]
        public void Connector_StoredState_UnsupportedSetters_Throw()
        {
            SimShip ship = Fc.Spawn(ConnectorCraft("ns", hullMass: 1000f), SpawnState.At(Vector3D.Zero));
            SimShipConnector con = Connector(ship);

            Assert.That(con.ThrowOut, Is.False);
            Assert.That(con.CollectAll, Is.False);
            // PullStrength reads the connector's Strength (the game's IMyShipConnector getter
            // returns Strength, MyShipConnector.cs:557-562); the template default is whole (1).
            Assert.That(con.PullStrength, Is.EqualTo(1f).Within(1e-6),
                "PullStrength must return the connector's Strength (default whole).");
            // IsParkingEnabled is stored (default true, the game's representation).
            Assert.That(con.IsParkingEnabled, Is.True);
            con.IsParkingEnabled = false; // a stored value, no throw
            Assert.That(con.IsParkingEnabled, Is.False);

            Assert.That((Action)(() => con.ThrowOut = true), Throws.TypeOf<NotSupportedException>()
                .With.Message.Contains("ThrowOut"));
            Assert.That((Action)(() => con.CollectAll = true), Throws.TypeOf<NotSupportedException>()
                .With.Message.Contains("CollectAll"));
            Assert.That((Action)(() => con.PullStrength = 1f), Throws.TypeOf<NotSupportedException>()
                .With.Message.Contains("PullStrength"));
        }

        // — the game's auto-detect gates —

        [Test]
        public void AutoDetect_Rejects_WhenFacesBeyondDetection()
        {
            // Faces 0.35 m apart: within the game's attach separation (0.35 sq ≈ 0.59 m) but
            // beyond this connector's detector sphere (its own block half-extent, 0.25 m), so
            // FindOtherConnector finds no partner and no approach begins.
            SimShip a = Fc.Spawn(ConnectorCraft("dda", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("ddb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.85, 0, 0))); // faces ~0.35 m apart
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step();

            Assert.That((ca.WorldPosition - cb.WorldPosition).Length(), Is.GreaterThan(ca.DetectorRadius),
                "Sanity: the faces must sit beyond this connector's detector radius.");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                "Faces beyond the detector radius must not be auto-attached.");
            Assert.That(ca.ApproachingInternal, Is.False);
        }

        [Test]
        public void AutoDetect_Rejects_WhenConnectAxesNotOpposed()
        {
            // Faces within detector range but both connect directions point the same way
            // (parallel, angle 0 < 3π/4) — the game's opposing-angle gate (MyShipConnector.cs:
            // 1488-1492) must reject. Collision disabled so the overlapping hulls don't matter.
            var world = new SimWorld(
                new SePhysicsProfile("NoCollide", collisionResponse: false), Vector3D.Zero);
            using (var ctx = new FleetTestContext(world))
            {
                SimShip a = ctx.Spawn(ConnectorCraft("aga", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
                SimShip b = ctx.Spawn(
                    ConnectorCraft("agb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))),
                    SpawnState.At(new Vector3D(0, 0, 0.2f))); // displaced perpendicular, faces ~0.2 m apart
                SimShipConnector ca = Connector(a);
                SimShipConnector cb = Connector(b);

                ctx.Step();

                Assert.That((ca.WorldPosition - cb.WorldPosition).Length(), Is.LessThanOrEqualTo(ca.DetectorRadius),
                    "Sanity: the faces must be within the detector radius so only the angle gate rejects.");
                Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                    "Parallel (non-opposed) connect directions must be rejected by the opposing-angle gate.");
            }
        }

        // — the full auto lifecycle: detect -> approach -> weld —

        [Test]
        public void AutoDock_Detects_Approaches_AndWeldsOnContact()
        {
            // A pair left near each other (faces within ~0.59 m and opposed) auto-detects each
            // tick, enters the approach via the malleable hinge (game TryAttach -> CreateConstraint),
            // and welds on contact (game Weld -> RecreateConstraintInternal) — with NO manual
            // Connect() at all.
            SimShip a = Fc.Spawn(ConnectorCraft("aa", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("ab", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0))); // faces ~0.18 m apart, within detection
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            // Not yet stepped: unpaired, unconnected.
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
            Assert.That(ca.IsConnected, Is.False);

            // One step: auto-detect pairs them via the malleable hinge (approach).
            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "On detection the pair must be paired (approaching) and Connectable.");
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
            Assert.That(ca.IsConnected, Is.False, "Approaching is not yet welded.");
            Assert.That(ca.ApproachingInternal, Is.True, "The pair must be in the approach stage.");
            Assert.That(ca.OtherConnector, Is.SameAs(cb), "Each connector must see the other as OtherConnector.");

            // The hinge draws them together and, on contact, welds.
            AutoWeld(ca);
            Assert.That(ca.IsConnected, Is.True, "On contact the pair must weld (fixed constraint).");
            Assert.That(cb.IsConnected, Is.True);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(ca.ApproachingInternal, Is.False, "Welding must end the approach stage.");
        }

        [Test]
        public void AutoDock_Connected_PairMovesAsOne()
        {
            // After the auto-weld the two grids are rigidly tied: an impulse to one moves both
            // together (the fixed constraint transfers the impulse).
            SimShip a = Fc.Spawn(ConnectorCraft("ma", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("mb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);
            AutoWeld(ca);
            Assert.That(ca.IsConnected, Is.True);

            Vector3 impulse = new Vector3(4000f, 0, 0);
            a.Grid.Body.ApplyLinearImpulse(impulse);
            Fc.Step(20);

            Vector3D va = a.Grid.Body.LinearVelocity;
            Vector3D vb = b.Grid.Body.LinearVelocity;
            Assert.That(va.Length(), Is.GreaterThan(0.5),
                "Applying an impulse to one grid must set that grid in motion.");
            Assert.That(vb.Length(), Is.GreaterThan(0.5),
                "The fixed constraint must transfer the impulse to the other grid (they move as one).");
            Assert.That((va - vb).Length(), Is.LessThan(0.4),
                "Two welded connectors must share (nearly) equal velocity.");
        }

        // — manual Connect / Disconnect on the game's gates —

        [Test]
        public void Connect_Welds_AnApproachingPair()
        {
            // The game's TryConnect (:919-936) welds a pair already in the approach
            // (InConstraint && !Connected). Put them within detection so they auto-pair, then
            // Connect() welds immediately.
            SimShip a = Fc.Spawn(ConnectorCraft("cw", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("cwb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step(); // auto-pair -> approaching, not yet welded
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
            Assert.That(ca.IsConnected, Is.False);

            ca.Connect(); // welds the approaching pair (game TryConnect -> Weld)
            Assert.That(ca.IsConnected, Is.True, "Connect must weld an approaching pair.");
            Assert.That(cb.IsConnected, Is.True);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(ca.OtherConnector, Is.SameAs(cb));
        }

        [Test]
        public void Connect_DoesNothing_WhenNotApproaching()
        {
            // Faces far apart: not auto-attached, so Connect() (game TryConnect) returns
            // immediately because !InConstraint.
            SimShip a = Fc.Spawn(ConnectorCraft("nc", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("ncb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(3.50, 0, 0))); // faces ~1 m apart
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            ca.Connect(); // must do nothing (not in a constraint)
            Assert.That(ca.IsConnected, Is.False);
            Assert.That(cb.IsConnected, Is.False);
            Assert.That(ca.OtherConnector, Is.Null);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            // And they were never tied: pushing A moves only A.
            a.Grid.Body.ApplyLinearImpulse(new Vector3(4000f, 0, 0));
            Fc.Step(5);
            Assert.That(a.Grid.Body.LinearVelocity.Length(), Is.GreaterThan(0.5));
            Assert.That(b.Grid.Body.LinearVelocity.Length(), Is.LessThan(1e-2),
                "Without a constraint, the other grid must not move.");
        }

        [Test]
        public void Disconnect_ReturnsBoth_ToCleanState_AndSeparatesGrids()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("dc", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("dcb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            AutoWeld(ca);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));

            ca.Disconnect();

            Assert.That(ca.IsConnected, Is.False, "Disconnect must clear the connected state.");
            Assert.That(cb.IsConnected, Is.False);
            Assert.That(ca.OtherConnector, Is.Null, "Disconnect must clear OtherConnector.");
            Assert.That(cb.OtherConnector, Is.Null);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            // The grids are independent again — no constraint ties them.
            b.SetState(SpawnState.At(new Vector3D(20, 0, 0)).ToBodyState(b.Grid.Body.Id));
            Fc.Step();
            a.Grid.Body.ApplyLinearImpulse(new Vector3(4000f, 0, 0));
            Fc.Step(5);
            Assert.That(a.Grid.Body.LinearVelocity.Length(), Is.GreaterThan(0.5));
            Assert.That(b.Grid.Body.LinearVelocity.Length(), Is.LessThan(1e-2),
                "After disconnect the two grids must move independently again.");
        }

        [Test]
        public void Approach_DriftsBeyondKeepRange_Detaches()
        {
            // An approaching (paired, not yet welded) pair whose faces are pulled apart beyond
            // the game's keep range (0.5 sq ≈ 0.7 m) comes apart (game :1571-1577 -> Detach).
            SimShip a = Fc.Spawn(ConnectorCraft("dk", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("dkb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);

            Fc.Step();
            Assert.That(ca.ApproachingInternal, Is.True, "Sanity: the pair must be approaching.");

            // Wrench B away beyond the keep range; the approach must break at the next tick.
            b.SetState(SpawnState.At(new Vector3D(6.0, 0, 0)).ToBodyState(b.Grid.Body.Id));
            Fc.Step();

            Assert.That(ca.ApproachingInternal, Is.False, "Drifting beyond the keep range must detach the pair.");
            Assert.That(ca.IsConnected, Is.False);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
            Assert.That(ca.OtherConnector, Is.Null);
        }

        [Test]
        public void Disconnect_Reconnect_IsDeterministic()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("rc", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("rcb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);

            Fc.Step();
            ca.Connect(); // weld the approaching pair
            Assert.That(ca.IsConnected, Is.True);
            ca.Disconnect();

            // After disconnect the pair stays at face contact, so on the next step auto-detect
            // re-attaches and, still at contact, re-welds cleanly (no residue from the prior
            // weld or disconnect).
            Fc.Step();
            Assert.That(ca.IsConnected, Is.True,
                "After disconnect the pair must re-attach and re-weld cleanly (faces still at contact).");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
        }

        // — toggle connect (flips a paired pair between connect and disconnect) —

        [Test]
        public void ToggleConnect_FlipsConnectedAndUnconnected()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("tg", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("tgb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);

            Fc.Step(); // approaching
            ca.ToggleConnect(); // game TrySwitch: approaching -> Connect (weld)
            Assert.That(ca.IsConnected, Is.True);

            ca.ToggleConnect(); // connected -> Disconnect
            Assert.That(ca.IsConnected, Is.False);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            // Net effect after toggling apart then auto-reattaching: the pair stays at face
            // contact, so the next tick re-attaches and, still at contact, re-welds — the
            // game's every-tick auto-reattach (UpdateBeforeSimulation10 -> TryAttach).
            Fc.Step();
            Assert.That(ca.IsConnected, Is.True,
                "After toggling off, an in-contact pair must auto-reattach and re-weld.");
        }

        // — status and working-state gating —

        [Test]
        public void Status_IsWorkingGates_Connectable()
        {
            // IsWorking (functional base) gates Connectable — the game's Status mapping
            // (MyShipConnector.cs:599-613) requires IsWorking && InConstraint for Connectable.
            SimShip a = Fc.Spawn(ConnectorCraft("wg", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("wgb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);

            ca.Enabled = false; // in range but not working: no auto-detect
            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                "A connector that is not working must not auto-pair even when in range.");
            Assert.That(ca.ApproachingInternal, Is.False);

            ca.Enabled = true;
            Fc.Step();
            Assert.That(ca.ApproachingInternal, Is.True,
                "Re-enabling the connector must allow auto-detect in range.");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
        }

        // — deterministic behavior across independent worlds —

        [Test]
        public void CaptureAndConnect_Deterministic_AcrossTwoWorlds()
        {
            var (first, second) = DeterminismHelper.RunTwice(RunDocking);

            Assert.That(first.Item2, Is.True, "The first world must have connected the pair.");
            Assert.That(second.Item2, Is.True, "The second world must have connected the pair (same outcome).");
            AssertDot(first.Item1, second.Item1, 1e-6,
                "The connected pair's post-step position must be identical across two independent worlds.");
        }

        private static Tuple<Vector3D, bool> RunDocking(FleetTestContext ctx)
        {
            SimShip a = ctx.Spawn(ConnectorCraft("da", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = ctx.Spawn(
                ConnectorCraft("db", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);
            ctx.Step();
            // Weld the approaching pair, then capture the outcome before disposing the world
            // (which disconnects the pair).
            int maxSteps = (int)Math.Ceiling(10.0 / TickClock.FixedStep);
            for (int i = 0; i < maxSteps && !ca.IsConnected; i++)
            {
                ctx.Step();
            }
            Vector3D pos = b.Grid.Body.PositionWorld;
            bool connected = ca.IsConnected;
            return Tuple.Create(pos, connected);
        }

        // — helpers —

        private static SimShipConnector Connector(SimShip ship)
        {
            return (SimShipConnector)ship.TerminalSystem.GetBlockWithName("C");
        }

        /// <summary>Steps the world until the connector and its partner weld (Connected) — the
        /// auto-flow, no manual Connect.</summary>
        private static void AutoWeld(SimShipConnector connector)
        {
            SimWorld world = connector.Grid.World;
            int maxSteps = (int)Math.Ceiling(8.0 / TickClock.FixedStep);
            for (int i = 0; i < maxSteps && !connector.IsConnected; i++)
            {
                world.Step();
            }
            Assert.That(connector.IsConnected,
                "The approach must converge and weld on contact within the deterministic timeout.");
        }

        /// <summary>A small-grid craft with a heavy hull and one connector on the face it faces;
        /// connOrient reorients the connect face.</summary>
        private static ShipTemplate ConnectorCraft(string name, float hullMass, Quaternion? connOrient = null,
            float reach = 0f, float strength = 1f)
        {
            Vector3I connCell;
            if (connOrient == null)
            {
                connCell = new Vector3I(2, 0, 0);
            }
            else
            {
                Vector3 fwd = Vector3.TransformNormal(
                    new Vector3(0, 0, -1f), Matrix.CreateFromQuaternion(connOrient.Value));
                // The dominant component of the connect forward: which
                // axis it points along and which direction.
                float ax = Math.Abs(fwd.X), ay = Math.Abs(fwd.Y), az = Math.Abs(fwd.Z);
                if (ax >= ay && ax >= az)
                {
                    connCell = new Vector3I((int)Math.Sign(fwd.X) * 2, 0, 0);
                }
                else if (ay >= az)
                {
                    connCell = new Vector3I(0, (int)Math.Sign(fwd.Y) * 2, 0);
                }
                else
                {
                    connCell = new Vector3I(0, 0, (int)Math.Sign(fwd.Z) * 2);
                }
            }
            return new ShipTemplate(name, 0.5f, new[]
            {
                new TemplateBlock("Hull", Vector3I.Zero, hullMass, 0.5f),
                new TemplateBlock("C", connCell, 50f, 0.5f,
                    kind: BlockKind.Connector,
                    connectorOrientation: connOrient, connectorReach: reach, connectorStrength: strength),
            }, maxLinearVelocity: 100f, maxAngularVelocity: 10f);
        }

        /// <summary>A rotation mapping the connection frame's local forward (0,0,-1) onto
        /// <paramref name="target"/>.</summary>
        private static Quaternion RotateForwardTo(Vector3 target)
        {
            Vector3 from = new Vector3(0, 0, -1f);
            target = Vector3.Normalize(target);
            float dot = Vector3.Dot(from, target);
            if (dot > 0.99999f)
            {
                return Quaternion.Identity;
            }
            if (dot < -0.99999f)
            {
                return Quaternion.CreateFromAxisAngle(Vector3.Right, (float)Math.PI);
            }
            Vector3 axis = Vector3.Cross(from, target);
            axis.Normalize();
            float angle = (float)Math.Acos(Clamp(dot, -1f, 1f));
            return Quaternion.CreateFromAxisAngle(axis, angle);
        }

        private static float Clamp(float v, float lo, float hi)
        {
            return v < lo ? lo : v > hi ? hi : v;
        }

        private static double mathDist(Vector3D a, Vector3D b)
        {
            return (a - b).Length();
        }

        private static void AssertDot(Vector3D a, Vector3D b, double tol, string message)
        {
            Assert.That((a - b).Length(), Is.LessThan(tol), message);
        }
    }
}
