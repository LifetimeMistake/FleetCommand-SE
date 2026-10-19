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
    /// approach-keep, and explicit welding via Connect (fixed constraint).</summary>
    [TestFixture]
    public class ConnectorConformanceTests : FleetTestFixture
    {
        [Test]
        public void SerializedConnection_RestoresWeldBeforeAutomaticDetection()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("load-a", 1000f,
                RotateForwardTo(Vector3.Right)), SpawnState.At(Vector3D.Zero));
            SimShip b = Fc.Spawn(ConnectorCraft("load-b", 1000f,
                RotateForwardTo(Vector3.Left)), SpawnState.At(new Vector3D(2.5, 0, 0)));
            SimShipConnector master = Connector(a);
            SimShipConnector slave = Connector(b);
            master.RestoreSerializedConnection(slave);
            Assert.That(master.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(slave.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(master.IsMaster, Is.True);
            Assert.That(master.OtherConnector, Is.SameAs(slave));
            Assert.Throws<ArgumentException>((Action)(() => master.RestoreSerializedConnection(master)));
            Assert.Throws<InvalidOperationException>((Action)(() => master.RestoreSerializedConnection(slave)));
            master.Disconnect();
            Assert.That(master.OtherConnector, Is.Null);
            Assert.That(slave.OtherConnector, Is.Null);
        }

        [Test]
        public void AutoAttach_RunsOnFourthWorkingUpdate10()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("cadence-a", 1000f,
                RotateForwardTo(Vector3.Right)), SpawnState.At(Vector3D.Zero));
            Fc.Spawn(ConnectorCraft("cadence-b", 1000f,
                RotateForwardTo(Vector3.Left)), SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector connector = Connector(a);
            Fc.Step(39);
            Assert.That(connector.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                "TryAttach is gated by ++m_update10Counter % 4 == 0 in UpdateBeforeSimulation10.");
            Fc.Step();
            Assert.That(connector.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
            connector.Enabled = false;
            Fc.Step(9);
            Assert.That(connector.InConstraint, Is.True,
                "The not-working detach also runs on UpdateBeforeSimulation10.");
            Fc.Step();
            Assert.That(connector.InConstraint, Is.False);
        }

        [Test]
        public void AutoAttach_AtContact_RemainsConnectableUntilExplicitConnect()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("contact-a", 1000f,
                RotateForwardTo(Vector3.Right)), SpawnState.At(Vector3D.Zero));
            SimShip b = Fc.Spawn(ConnectorCraft("contact-b", 1000f,
                RotateForwardTo(Vector3.Left)), SpawnState.At(new Vector3D(2.55, 0, 0)));
            SimShipConnector connector = Connector(a);
            Fc.Step(120);
            Assert.That(connector.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "MyShipConnector.UpdateBeforeSimulation10 never calls Weld; contact only holds an approach constraint.");
            connector.Connect();
            Assert.That(connector.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
        }

        [TestCase("ConnectorSmall", -0.335196674f, 0.1383981705f)]
        [TestCase("Connector", -1.31683457f, 0.6155442f)]
        [TestCase("ConnectorMedium", -0.5154143f, 0.636063575f)]
        public void ImportedConnector_UsesModelDummyMatingPoint(string subtype, float depth, float radius)
        {
            var block = new TemplateBlock("C", Vector3I.Zero, 100f, 0.5f,
                kind: BlockKind.Connector, connectorDetectorDummy: ConnectorDetectorGeometry.ForSubtype(subtype));
            var template = new ShipTemplate(subtype, 0.5f, new[] { block });
            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            SimShipConnector connector = Connector(ship);
            AssertDot(connector.WorldPosition, new Vector3D(0, 0, depth), 1e-6,
                "ConstraintPositionInGridSpace uses the dominant-axis projection of model dummy translation.");
            AssertDot(connector.ConnectAxisGrid, Vector3D.Forward, 1e-6,
                "ConstraintAxisGridSpace points from block centre to the projected dummy translation.");
            Assert.That(connector.DetectorRadius, Is.EqualTo(radius).Within(1e-6));
        }

        [Test]
        public void ConnectorFrame_ComposesLocalRotationsBeforeGridRotation()
        {
            Quaternion blockRotation = Quaternion.CreateFromAxisAngle(Vector3.Right, MathHelper.PiOver2);
            Quaternion connectorRotation = Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver2);
            var block = new TemplateBlock("C", Vector3I.Zero, 100f, 0.5f,
                orientation: blockRotation, kind: BlockKind.Connector, connectorOrientation: connectorRotation);
            var template = new ShipTemplate("rotated", 0.5f, new[] { block });
            MatrixD gridRotation = MatrixD.CreateRotationZ(MathHelper.PiOver2);
            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero, gridRotation));
            Matrix expected = Matrix.CreateFromQuaternion(connectorRotation) * Matrix.CreateFromQuaternion(blockRotation);
            AssertDot(Connector(ship).ConnectAxisGrid, Vector3.TransformNormal(Vector3.Backward, expected), 1e-6,
                "VRage uses row vectors: connector-local rotation precedes block-local rotation.");
            AssertDot(Connector(ship).ConnectAxisWorld,
                Vector3D.TransformNormal(Vector3.TransformNormal(Vector3.Backward, expected), gridRotation), 1e-6,
                "Grid rotation must follow the local connector and block rotations.");
        }

        [TestCase(false, true)]
        [TestCase(true, false)]
        public void ConnectorPairing_UsesPortSizeRatherThanGridSize(bool smallPort, bool expectedPairing)
        {
            var smallBlock = new TemplateBlock("C", Vector3I.Zero, 100f, 0.5f,
                kind: BlockKind.Connector, connectorDetectorDummy: ConnectorDetectorGeometry.ForSubtype("ConnectorMedium"));
            smallBlock.ConnectorIsSmall = smallPort;
            var largeBlock = new TemplateBlock("C", Vector3I.Zero, 100f, 2.5f,
                orientation: Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.Pi),
                kind: BlockKind.Connector, connectorDetectorDummy: ConnectorDetectorGeometry.ForSubtype("Connector"));
            largeBlock.ConnectorIsSmall = false;
            SimShip small = Fc.Spawn(new ShipTemplate("small", 0.5f, new[] { smallBlock }),
                SpawnState.At(new Vector3D(0, 0, 1.88224887)));
            Fc.Spawn(new ShipTemplate("large", 2.5f, new[] { largeBlock }), SpawnState.At(Vector3D.Zero));
            Fc.Step(40);
            Assert.That(Connector(small).Status == MyShipConnectorStatus.Connectable, Is.EqualTo(expectedPairing),
                "MyShipConnector.TryAttach compares IsSmallConnector from dummy names, independent of grid size.");
        }

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
            // block centre along the connect axis by the block's half-extent
            // (0.25) — the game's centre + dominant-axis face projection
            // (MyShipConnector.cs:1944-1952).
            Vector3D expected = new Vector3D(0, 0, 7) + new Vector3D(1, 0, 0) + new Vector3D(0, 0, 0.25);
            AssertDot(con.WorldPosition, expected, 1e-3,
                "The connection point must be the block's world position offset to the mating face.");

            // Default connect axis (identity mount, block identity) is the game's default
            // attach direction: ConnectDirection = Vector3.Forward (0,0,-1) maps through
            // a = CD.X·Left + CD.Y·Up + CD.Z·Forward = (0,0,1) (MyShipConnector.cs:1489-1490).
            AssertDot(con.ConnectAxisWorld, new Vector3D(0, 0, 1), 1e-3,
                "An identity-mounted connector must extend along the game's default attach direction (0,0,1).");
        }

        [Test]
        public void ConnectDirection_SelectsFaceLikeTheGame()
        {
            // ConnectDirection is the game's six-face attach-direction selector
            // (MyShipConnector.cs:1489-1490): a = CD.X·Left + .Y·Up + .Z·Forward in the block
            // frame. The structural platform connector's (0,-1,0) must connect along -Up; the
            // Docking craft's (-1,0,0) along +X.
            SimShip down = Fc.Spawn(
                ConnectorCraft("cdd", hullMass: 1000f, connectDirection: new Vector3(0, -1, 0)),
                SpawnState.At(Vector3D.Zero));
            AssertDot(Connector(down).ConnectAxisWorld, new Vector3D(0, -1, 0), 1e-3,
                "(0,-1,0) must map to -Up (the platform connector's downward attach face).");

            SimShip posx = Fc.Spawn(
                ConnectorCraft("cdp", hullMass: 1000f, connectDirection: new Vector3(-1, 0, 0)),
                SpawnState.At(new Vector3D(60, 0, 0)));
            AssertDot(Connector(posx).ConnectAxisWorld, new Vector3D(1, 0, 0), 1e-3,
                "(-1,0,0) must map to +X (CD.X·Left = (-1)·Left = +X).");
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

        // — stored scenario state: game-faithful stored values; setters store, not throw —

        [Test]
        public void Connector_StoredState_SettersStore_LikeTheGame()
        {
            SimShip ship = Fc.Spawn(ConnectorCraft("ns", hullMass: 1000f), SpawnState.At(Vector3D.Zero));
            SimShipConnector con = Connector(ship);

            // ThrowOut / CollectAll default false, stored as plain values exactly like the game's
            // IMyShipConnector setters (MyShipConnector.cs:533-555) — the magnetic inventory
            // effect is not modeled, so they only hold the value.
            Assert.That(con.ThrowOut, Is.False);
            con.ThrowOut = true;
            Assert.That(con.ThrowOut, Is.True, "ThrowOut must store its value (MyShipConnector.cs:533-543).");
            con.ThrowOut = false;
            Assert.That(con.ThrowOut, Is.False);
            Assert.That(con.CollectAll, Is.False);
            con.CollectAll = true;
            Assert.That(con.CollectAll, Is.True, "CollectAll must store its value (MyShipConnector.cs:545-555).");

            // PullStrength reads the connector's Strength (MyShipConnector.cs:557-562); the setter
            // clamps to [1e-6, 1] and stores (MyShipConnector.cs:563-570). The template default
            // is whole (1).
            Assert.That(con.PullStrength, Is.EqualTo(1f).Within(1e-6),
                "PullStrength must return the connector's Strength (default whole).");
            con.PullStrength = 0.5f;
            Assert.That(con.PullStrength, Is.EqualTo(0.5f).Within(1e-6),
                "PullStrength must store a clamped value (the game stores the clamped Strength).");
            con.PullStrength = 5f;  // above 1 → clamped to 1
            Assert.That(con.PullStrength, Is.EqualTo(1f).Within(1e-6),
                "PullStrength clamps to 1 (the game's MathHelper.Clamp(value, 1e-6, 1)).");
            con.PullStrength = 0f; // below 1e-6 → clamped to 1e-6
            Assert.That(con.PullStrength, Is.EqualTo(1e-6f).Within(1e-9),
                "PullStrength clamps to 1e-6 (the game's lower clamp).");

            // IsParkingEnabled is stored (default true, the game's representation).
            Assert.That(con.IsParkingEnabled, Is.True);
            con.IsParkingEnabled = false; // a stored value, no throw
            Assert.That(con.IsParkingEnabled, Is.False);
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

            Fc.Step(40);

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

                Assert.That((ca.WorldPosition - cb.WorldPosition).Length(), Is.LessThanOrEqualTo(ca.DetectorRadius),
                    "Sanity: the faces must be within the detector radius so only the angle gate rejects.");
                ctx.Step(40);
                Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                    "Parallel (non-opposed) connect directions must be rejected by the opposing-angle gate.");
            }
        }

        // — the full auto lifecycle: detect -> approach -> weld —

        [Test]
        public void AutoAttach_DetectsAndApproaches_ExplicitConnectWelds()
        {
            // Nearby opposed connectors auto-attach through a malleable hinge.
            // Explicit Connect replaces that approach with the welded fixed constraint.
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
            Fc.Step(40);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "On detection the pair must be paired (approaching) and Connectable.");
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
            Assert.That(ca.IsConnected, Is.False, "Approaching is not yet welded.");
            Assert.That(ca.ApproachingInternal, Is.True, "The pair must be in the approach stage.");
            Assert.That(ca.OtherConnector, Is.SameAs(cb), "Each connector must see the other as OtherConnector.");

            // Explicitly weld the approaching pair.
            ConnectPair(ca);
            Assert.That(ca.IsConnected, Is.True, "Explicit Connect must weld (fixed constraint).");
            Assert.That(cb.IsConnected, Is.True);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(ca.ApproachingInternal, Is.False, "Welding must end the approach stage.");
        }

        [Test]
        public void ExplicitConnect_PairMovesAsOne()
        {
            // After explicit Connect the two grids are rigidly tied: an impulse to one moves both
            // together (the fixed constraint transfers the impulse).
            SimShip a = Fc.Spawn(ConnectorCraft("ma", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("mb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(2.68, 0, 0)));
            SimShipConnector ca = Connector(a);
            ConnectPair(ca);
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

            Fc.Step(40); // auto-pair -> approaching, not yet welded
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

            Fc.Step(40);
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

            ConnectPair(ca);
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
            Fc.Step(40);
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

            Fc.Step(40);
            Assert.That(ca.ApproachingInternal, Is.True, "Sanity: the pair must be approaching.");

            // Wrench B away beyond the keep range; the approach must break at the next tick.
            Fc.Step(9);
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

            Fc.Step(40);
            ca.Connect(); // weld the approaching pair
            Assert.That(ca.IsConnected, Is.True);
            ca.Disconnect();

            // Auto-detection restores the approach; explicit Connect redocks the pair.
            Fc.Step(40);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
            Assert.That(ca.IsConnected, Is.False);
            ca.Connect();
            Assert.That(ca.IsConnected, Is.True,
                "Explicit Connect must redock the pair.");
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

            Fc.Step(40); // approaching
            ca.ToggleConnect(); // game TrySwitch: approaching -> Connect (weld)
            Assert.That(ca.IsConnected, Is.True);

            ca.ToggleConnect(); // connected -> Disconnect
            Assert.That(ca.IsConnected, Is.False);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            // Auto-reattachment leaves the pair Connectable until explicitly toggled.
            Fc.Step(40);
            Assert.That(ca.IsConnected, Is.False);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
            ca.ToggleConnect();
            Assert.That(ca.IsConnected, Is.True);
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
            Fc.Step(40);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                "A connector that is not working must not auto-pair even when in range.");
            Assert.That(ca.ApproachingInternal, Is.False);

            ca.Enabled = true;
            Fc.Step(40);
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
            ctx.Step(40);
            ca.Connect();
            // Weld the approaching pair, then capture the outcome before disposing the world
            // (which disconnects the pair).
            int maxSteps = (int)Math.Ceiling(10.0 / TickClock.FixedStep);
            for (int i = 0; i < maxSteps && !ca.IsConnected; i++)
            {
                ctx.Step(40);
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

        /// <summary>Waits for approach and explicitly connects the pair.</summary>
        private static void ConnectPair(SimShipConnector connector)
        {
            SimWorld world = connector.Grid.World;
            int maxSteps = (int)Math.Ceiling(8.0 / TickClock.FixedStep);
            for (int i = 0; i < maxSteps && connector.Status != MyShipConnectorStatus.Connectable; i++)
            {
                world.Step(40);
            }
            Assert.That(connector.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));
            connector.Connect();
            Assert.That(connector.IsConnected, "Explicit Connect must weld the pair.");
        }

        /// <summary>A small-grid craft with a heavy hull and one connector on the face it faces;
        /// connOrient reorients the connect face.</summary>
        private static ShipTemplate ConnectorCraft(string name, float hullMass, Quaternion? connOrient = null,
            float reach = 0f, float strength = 1f, Vector3? connectDirection = null)
        {
            Vector3I connCell;
            if (connOrient == null)
            {
                connCell = new Vector3I(2, 0, 0);
            }
            else
            {
                Vector3 fwd = Vector3.TransformNormal(
                    new Vector3(0, 0, 1f), Matrix.CreateFromQuaternion(connOrient.Value));
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
                    connectorOrientation: connOrient, connectorReach: reach, connectorStrength: strength,
                    connectDirection: connectDirection),
            }, maxLinearVelocity: 100f, maxAngularVelocity: 10f);
        }

        /// <summary>A rotation mapping the connection frame's block-local attach direction
        /// (0,0,1) onto <paramref name="target"/>.</summary>
        private static Quaternion RotateForwardTo(Vector3 target)
        {
            Vector3 from = new Vector3(0, 0, 1f);
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
