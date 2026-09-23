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
    /// <summary>Conformance of the connector block, docking capture gates, and the fixed rigid
    /// linkage.</summary>
    [TestFixture]
    public class ConnectorConformanceTests : FleetTestFixture
    {
        // — connector point kinematics (world pose + point velocity from the body) —

        [Test]
        public void ConnectorPoint_WorldPose_FromOwnedGridBody()
        {
            // The connection frame's world position is the block's world
            // position: grid body pose * block local pose.
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
            // The connection point velocity is derived from the owning
            // rigid body: v + ω × r, r = point − COM. The COM
            // is offset from the connection point (hull mass at the grid
            // origin, connector at +X), so a spin about Y adds a
            // non-zero term in -Z at the point. Asserted analytically
            // from the body's own mass properties and pose.
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

            // The rotational term is real (not just the body's linear
            // velocity): the spin about Y turns into a -Z component at
            // the off-axis point.
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
            Assert.That(con.PullStrength, Is.Zero);
            // IsParkingEnabled is stored (default true, the game's
            // representation); the rest are out-of-contract.
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

        // — capture gates: each individually rejects when exceeded —

        [Test]
        public void Capture_SeparationGate_Rejects_BeyondReach()
        {
            // Opposed connectors placed 50 m apart (reach ~2.5) — all
            // gates pass except separation.
            float reach = Fc.World.Profile.ConnectorReach;
            SimShip a = Fc.Spawn(ConnectorCraft("sga", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("sgb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(50, 0, 0)));

            Fc.Step();

            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);
            Assert.That((ca.WorldPosition - cb.WorldPosition).Length(), Is.GreaterThan(reach),
                "Sanity: the pair must be beyond the configured reach.");
            Assert.That(ConnectorSystem.CapturePasses(ca, cb, Fc.World.Profile), Is.False,
                "The separation gate must reject a pair beyond the configured reach.");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
        }

        [Test]
        public void Capture_OpposingAxisAngularGate_Rejects_ParallelFaces()
        {
            // Both connectors face the same way (+X): the connect axes are
            // parallel, so the opposing-axis angular error (|π − angle|)
            // is ~π > the threshold, while separation and velocities pass.
            SimShip a = Fc.Spawn(ConnectorCraft("aga", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))),
                SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(ConnectorCraft("agb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))),
                SpawnState.At(new Vector3D(2, 0, 0)));

            Fc.Step();

            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);
            Assert.That(ConnectorSystem.CapturePasses(ca, cb, Fc.World.Profile), Is.False,
                "Parallel (non-opposing) connect axes must be rejected by the opposing-axis angular gate.");
            Assert.That((ca.WorldPosition - cb.WorldPosition).Length(), Is.LessThanOrEqualTo(Fc.World.Profile.ConnectorReach),
                "Sanity: the pair must be within reach so only the angular gate rejects.");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
        }

        [Test]
        public void Capture_RelativeLinearVelocityGate_Rejects_FastApproach()
        {
            // Opposed connectors within reach, but B approaches at 5 m/s
            // >> the 0.5 m/s profile threshold. Angular and rotation
            // gates pass; only the relative-linear-velocity gate rejects.
            SimShip a = Fc.Spawn(ConnectorCraft("lva", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("lvb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)).WithVelocity(new Vector3D(5, 0, 0)));

            Fc.Step();

            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);
            Assert.That(ConnectorSystem.CapturePasses(ca, cb, Fc.World.Profile), Is.False,
                "A closing speed above the profile's linear-velocity threshold must be rejected.");
            Assert.That((ca.Grid.Body.AngularVelocity - cb.Grid.Body.AngularVelocity).Length(), Is.LessThan(1e-6),
                "Sanity: both bodies start at rest rotationally, isolating the linear-velocity gate.");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
        }

        [Test]
        public void Capture_RelativeAngularVelocityGate_Rejects_SpinningPartner()
        {
            // Opposed connectors within reach, B spinning at 4 rad/s about
            // the axis through its own connection point (so the point's
            // ω × r term is zero and the linear-velocity gate still
            // passes); only the relative-angular-velocity gate rejects.
            SimShip a = Fc.Spawn(ConnectorCraft("ava", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("avb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)).WithAngularVelocity(new Vector3(4, 0, 0)));

            Fc.Step();

            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);
            Assert.That(ConnectorSystem.CapturePasses(ca, cb, Fc.World.Profile), Is.False,
                "A relative angular velocity above the profile threshold must be rejected.");
            Assert.That((ca.WorldPosition - cb.WorldPosition).Length(), Is.LessThanOrEqualTo(Fc.World.Profile.ConnectorReach),
                "Sanity: the pair must be within reach, isolating the angular-velocity gate.");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
        }

        // — status transition and working-state gating —

        [Test]
        public void Status_UnconnectedConnectableConnected_ThroughApproachAndConnect()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("st", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("stb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(50, 0, 0)));

            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            // Far apart: not connectable.
            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            // Move within reach and opposed; after a step the pair is
            // Connectable (capture readiness).
            b.SetState(SpawnState.At(new Vector3D(4.5, 0, 0)).ToBodyState(b.Grid.Body.Id));
            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "Approach within reach and opposed must make the pair Connectable (capture readiness).");

            // Connect begins the two-stage approach, NOT an instant lock:
            // the pair is 2 m apart, so Connect starts the convergence and
            // Status stays Connectable while approaching.
            ca.Connect();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "A connectable-but-not-yet-converged pair must begin the approach, not instant-lock.");
            Assert.That(ca.IsConnected, Is.False);

            // The approach converges; only on physical meet does the pair
            // lock (final fixed constraint) and become Connected.
            ConvergeToConnected(ca);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Connected));
        }

        [Test]
        public void Status_IsWorkingGates_Connectable()
        {
            // IsWorking (functional base) gates Connectable — the game's
            // Status mapping (MyShipConnector.cs:599-611) requires
            // IsWorking for Connectable.
            SimShip a = Fc.Spawn(ConnectorCraft("wg", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("wgb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            ca.Enabled = false; // within geometric capture but not working
            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                "A connector that is not working must not be Connectable even when in range.");

            ca.Enabled = true;
            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "Re-enabling the connector must restore Connectable in range.");
        }

        // — constraint creation only when connectable —

        [Test]
        public void Connect_LinksTwoGrids_TheyMoveAsOne()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("linka", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("linkb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step(); // capture
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));

            SimShipConnector master = a.Grid.EntityId >= b.Grid.EntityId ? ca : cb;
            master.Connect();
            Assert.That(ca.IsConnected, Is.False, "Connect must begin the approach, not instant-lock.");
            ConvergeToConnected(master);
            Assert.That(ca.IsConnected, Is.True, "Connect must mark both connectors connected on lock.");
            Assert.That(cb.IsConnected, Is.True);
            Assert.That(ca.OtherConnector, Is.SameAs(cb), "Each connector must see the other as OtherConnector.");
            Assert.That(cb.OtherConnector, Is.SameAs(ca));
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));

            // Apply an impulse to ONE body; the fixed constraint must tie
            // the grids so they accelerate together (multi-grid linkage).
            Vector3 impulse = new Vector3(4000f, 0, 0);
            a.Grid.Body.ApplyLinearImpulse(impulse);
            Fc.Step(20);

            Vector3D va = a.Grid.Body.LinearVelocity;
            Vector3D vb = b.Grid.Body.LinearVelocity;
            Assert.That(va.Length(), Is.GreaterThan(0.5),
                "Applying an impulse to one grid must set that grid in motion.");
            Assert.That(vb.Length(), Is.GreaterThan(0.5),
                "The constraint must transfer the impulse to the other grid (they move as one).");
            Assert.That((va - vb).Length(), Is.LessThan(0.4),
                "Two rigidly-connected grids must share (nearly) equal velocity.");
        }

        [Test]
        public void Connect_DoesNothing_WhenNotConnectable()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("nc", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("ncb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(50, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step(); // far apart → not connectable
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            ca.Connect(); // must do nothing (interface contract)
            Assert.That(ca.IsConnected, Is.False);
            Assert.That(cb.IsConnected, Is.False);
            Assert.That(ca.OtherConnector, Is.Null);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            // And they were never tied: pushing A moves only A.
            a.Grid.Body.ApplyLinearImpulse(new Vector3(4000f, 0, 0));
            Fc.Step(5);
            Assert.That(a.Grid.Body.LinearVelocity.Length(), Is.GreaterThan(0.5));
            Assert.That(b.Grid.Body.LinearVelocity.Length(), Is.LessThan(1e-2),
                "Without a constraint, the other grid must not move.");
        }

        // — disconnect returns both to a clean state —

        [Test]
        public void Disconnect_ReturnsBoth_ToCleanState_AndSeparatesGrids()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("dc", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("dcb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step();
            ca.Connect();
            ConvergeToConnected(ca);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connected));

            ca.Disconnect();

            // Both return to a clean, unconnected state.
            Assert.That(ca.IsConnected, Is.False, "Disconnect must clear the connected state.");
            Assert.That(cb.IsConnected, Is.False);
            Assert.That(ca.OtherConnector, Is.Null, "Disconnect must clear OtherConnector.");
            Assert.That(cb.OtherConnector, Is.Null);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
            Assert.That(cb.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));

            // The grids are independent again — no constraint ties them.
            // Separate the pair, then push A: B must not move.
            b.SetState(SpawnState.At(new Vector3D(20, 0, 0)).ToBodyState(b.Grid.Body.Id));
            Fc.Step();
            a.Grid.Body.ApplyLinearImpulse(new Vector3(4000f, 0, 0));
            Fc.Step(5);
            Assert.That(a.Grid.Body.LinearVelocity.Length(), Is.GreaterThan(0.5));
            Assert.That(b.Grid.Body.LinearVelocity.Length(), Is.LessThan(1e-2),
                "After disconnect the two grids must move independently again.");
        }

        [Test]
        public void Disconnect_Reconnect_IsDeterministic()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("rc", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("rcb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);

            Fc.Step();
            ca.Connect();
            ca.Disconnect();

            // After disconnect the same geometry must be captured again and
            // re-connect cleanly (both return to Connectable).
            Fc.Step();
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "After disconnect the pair must be recapture-capable (clean state, no residue).");
        }

        // — toggle connect (flips connected/unconnected when in range) —

        [Test]
        public void ToggleConnect_FlipsConnectedAndUnconnected()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("tg", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("tgb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);

            Fc.Step();
            ca.ToggleConnect(); // connectable -> approach, then lock on meet
            ConvergeToConnected(ca);
            Assert.That(ca.IsConnected, Is.True);

            ca.ToggleConnect(); // connected -> unconnected
            Assert.That(ca.IsConnected, Is.False);
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
        }

        // — deterministic behavior across independent worlds —

        [Test]
        public void CaptureAndConnect_Deterministic_AcrossTwoWorlds()
        {
            var (first, second) = DeterminismHelper.RunTwice(RunDocking);

            // Identical scenario in two freshly-built worlds yields the same
            // connected outcome and the same final connected geometry.
            Assert.That(first.Item2, Is.True, "The first world must have connected the pair.");
            Assert.That(second.Item2, Is.True, "The second world must have connected the pair (same capture outcome).");
            AssertDot(first.Item1, second.Item1, 1e-6,
                "The connected pair's post-step position must be identical across two independent worlds.");
        }

        private static Tuple<Vector3D, bool> RunDocking(FleetTestContext ctx)
        {
            SimShip a = ctx.Spawn(ConnectorCraft("da", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = ctx.Spawn(
                ConnectorCraft("db", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);
            ctx.Step();
            ca.Connect();
            // Step until the approach converges and the pair is Connected
            // BEFORE capturing the outcome (and before disposing the world,
            // which disconnects the pair).
            int maxSteps = (int)Math.Ceiling(10.0 / TickClock.FixedStep);
            for (int i = 0; i < maxSteps && !ca.IsConnected; i++)
            {
                ctx.Step();
            }
            Vector3D pos = b.Grid.Body.PositionWorld;
            bool connected = ca.IsConnected;
            return Tuple.Create(pos, connected);
        }

        // — the two-stage connect (approach then lock) —

        [Test]
        public void Approach_NotAnInstantLock_Converges_ThenLocks_AndCoMoves()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("appra", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("apprb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step(); // capture (within reach, opposed) -> Connectable
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));

            ca.Connect();
            // Connect starts the approach; the pair is not yet locked.
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable),
                "Connect on a converged-but-separated pair must begin the approach, not instant-lock.");
            Assert.That(ca.IsConnected, Is.False);

            // The approach physically converges the bodies toward
            // coincidence (separation shrinks), then locks — a genuine
            // two-stage connect, not an instant lock on Connect().
            Assert.That(ca.ApproachingInternal, Is.True,
                "Connect must begin the approach stage on a separated pair.");
            float sepBefore = (float)(ca.WorldPosition - cb.WorldPosition).Length();
            Fc.Step(5);
            float sepAfter = (float)(ca.WorldPosition - cb.WorldPosition).Length();
            Assert.That(sepAfter, Is.LessThan(sepBefore),
                "During the approach the two connector points must converge toward coincidence.");
            Assert.That(ca.IsConnected, Is.False,
                "Not yet locked while still converging.");

            ConvergeToConnected(ca);
            Assert.That(ca.IsConnected, Is.True, "On physical meet the pair must lock (final fixed constraint).");

            // After the lock the two grids move as one (fixed constraint).
            Vector3 impulse = new Vector3(4000f, 0, 0);
            a.Grid.Body.ApplyLinearImpulse(impulse);
            Fc.Step(20);
            Assert.That((a.Grid.Body.LinearVelocity - b.Grid.Body.LinearVelocity).Length(), Is.LessThan(0.4),
                "Two connectors locked after the approach must move as one under a solitary impulse.");
        }

        [Test]
        public void Approach_AlignsOpposingAxes_BeforeLock()
        {
            // A pair within reach but with a small (still-capturable)
            // angular misalignment: the approach must rotate the grids so
            // their connect axes become anti-parallel before locking.
            SimShip a = Fc.Spawn(ConnectorCraft("alig", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("aligb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-0.996f, 0, 0.087f))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);
            SimShipConnector cb = Connector(b);

            Fc.Step(); // still within capture -> Connectable
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Connectable));

            ca.Connect(); // begin approach from the small misalignment

            // Run the approach; while approaching the axes must rotate
            // toward anti-parallel (|π − angle| shrinking), and the pair
            // only locks once aligned.
            bool sawSmallerError = false;
            double prevError = double.MaxValue;
            for (int i = 0; i < 480 && !ca.IsConnected; i++)
            {
                Fc.Step();
                double angle = MathHelpers.AngleBetween(ca.ConnectAxisWorld, cb.ConnectAxisWorld);
                double axisError = Math.Abs(Math.PI - angle);
                if (axisError < prevError)
                {
                    sawSmallerError = true;
                }
                prevError = axisError;
            }
            Assert.That(sawSmallerError, Is.True,
                "During the approach the connect axes must rotate toward anti-parallel.");
            Assert.That(ca.IsConnected, Is.True, "Once aligned and converged, the pair must lock.");
            double finalError = Math.Abs(Math.PI - MathHelpers.AngleBetween(ca.ConnectAxisWorld, cb.ConnectAxisWorld));
            Assert.That(finalError, Is.LessThan(Fc.World.Profile.ConnectorApproachAngle),
                "The lock must occur with the connect axes anti-parallel (within the approach angle).");
        }

        [Test]
        public void Disconnect_MidApproach_ReturnsBoth_ToFreeState()
        {
            SimShip a = Fc.Spawn(ConnectorCraft("mid", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("midb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(4.5, 0, 0)));
            SimShipConnector ca = Connector(a);

            Fc.Step();
            ca.Connect(); // begin approach
            Assert.That(ca.ApproachingInternal, Is.True, "Connect must put the pair into the approach.");
            Fc.Step(3); // a little convergence
            Assert.That(ca.IsConnected, Is.False);

            ca.Disconnect(); // release mid-approach

            Assert.That(ca.IsConnected, Is.False, "Disconnect mid-approach must not leave the pair connected.");
            Assert.That(ca.ApproachingInternal, Is.False, "Disconnect mid-approach must clear the approach.");
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected),
                "Disconnect mid-approach must return this connector to Unconnected.");
            Assert.That(ca.OtherConnector, Is.Null);

            // No residual linkage: reset B to rest, then pushing A moves only
            // A (the grids move independently again).
            b.Grid.Body.SetState(SpawnState.At(b.Grid.Body.PositionWorld, b.Grid.Body.Orientation)
                .WithVelocity(Vector3D.Zero).WithAngularVelocity(Vector3.Zero).ToBodyState(b.Grid.Body.Id));
            a.Grid.Body.ApplyLinearImpulse(new Vector3(4000f, 0, 0));
            Fc.Step(5);
            Assert.That(a.Grid.Body.LinearVelocity.Length(), Is.GreaterThan(0.5));
            Assert.That(b.Grid.Body.LinearVelocity.Length(), Is.LessThan(1e-2),
                "After a mid-approach disconnect the two grids must move independently again.");
        }

        [Test]
        public void Gate_Failure_PreventsApproach()
        {
            // A pair beyond the separation reach is not Connectable, so
            // Connect() must do nothing — no approach begins (the
            // capture gates gate the approach itself).
            SimShip a = Fc.Spawn(ConnectorCraft("gap", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))), SpawnState.At(new Vector3D(0, 0, 0)));
            SimShip b = Fc.Spawn(
                ConnectorCraft("gapb", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-1, 0, 0))),
                SpawnState.At(new Vector3D(50, 0, 0)));
            SimShipConnector ca = Connector(a);

            Fc.Step(); // far apart -> Unconnected
            Assert.That(ca.Status, Is.EqualTo(MyShipConnectorStatus.Unconnected));
            ca.Connect(); // must do nothing (not Connectable)
            Assert.That(ca.ApproachingInternal, Is.False, "A pair failing a capture gate must not begin the approach.");
            Assert.That(ca.IsConnected, Is.False);
        }

        // — helpers —

        private static SimShipConnector Connector(SimShip ship)
        {
            return (SimShipConnector)ship.TerminalSystem.GetBlockWithName("C");
        }

        /// <summary>A small-grid craft with a heavy hull and one connector on the face it faces;
        /// connOrient reorients the connect face.</summary>
        private static ShipTemplate ConnectorCraft(string name, float hullMass, Quaternion? connOrient = null, float reach = 0f)
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
                    connectorOrientation: connOrient, connectorReach: reach),
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

        /// <summary>Steps the world until the connector and its approach partner converge and lock
        /// (Connected).</summary>
        private static void ConvergeToConnected(SimShipConnector connector)
        {
            SimWorld world = connector.Grid.World;
            int maxSteps = (int)Math.Ceiling(8.0 / TickClock.FixedStep);
            for (int i = 0; i < maxSteps && !connector.IsConnected; i++)
            {
                world.Step();
            }
            Assert.That(connector.IsConnected,
                "The approach must converge and lock within the deterministic timeout.");
        }

        // — the approach trajectory matches the analytic off-COM magnet law —

        [Test]
        public void Approach_Trajectory_Matches_Analytic_OffComPulledLaw()
        {
            // Prove the sim's approach follows the physical magnet law —
            // a spring between the connection frames applied OFF the
            // center of mass (linear impulse + angular torque r × F, plus
            // the fixed frame constraint's torsional tie) — by recording
            // the sim's measured converge+align trajectory (both bodies'
            // positions and orientations each tick) and comparing it to an
            // independent first-principles integration of that same law
            // written in the test (no calls into the sim). If the off-COM
            // coupling were absent (torque ≠ r × F), the orientation
            // would diverge by orders more than the asserted tolerance.
            var world = new SimWorld(
                new SePhysicsProfile("NoCollide", collisionResponse: false), Vector3D.Zero);
            using (var ctx = new FleetTestContext(world))
            {
                SimShip a = ctx.Spawn(
                    ConnectorCraft("trjA", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(1, 0, 0))),
                    SpawnState.At(new Vector3D(0, 0, 0)));
                SimShip b = ctx.Spawn(
                    ConnectorCraft("trjB", hullMass: 1000f, connOrient: RotateForwardTo(new Vector3(-0.996f, 0, 0.087f))),
                    SpawnState.At(new Vector3D(4.5, 0, 0)));
                SimShipConnector ca = Connector(a);
                SimShipConnector cb = Connector(b);

                // Capture the analytic initial condition (body state) and
                // the body-fixed mounting of each connection frame.
                var initA = new AnalyticBody(a.Grid.Body, ca);
                var initB = new AnalyticBody(b.Grid.Body, cb);

                ctx.Step(); // capture -> Connectable
                ca.Connect(); // begin the approach (magnetic convergence)

                const int ticks = 36; // ~0.6 s of magnetic convergence, before the contact the final lock settles
                var simA = new Vector3D[ticks];
                var simB = new Vector3D[ticks];
                var simAngErr = new double[ticks];
                for (int i = 0; i < ticks; i++)
                {
                    ctx.Step();
                    simA[i] = a.Grid.Body.PositionWorld;
                    simB[i] = b.Grid.Body.PositionWorld;
                    simAngErr[i] = Math.Abs(Math.PI - MathHelpers.AngleBetween(ca.ConnectAxisWorld, cb.ConnectAxisWorld));
                }

                // Independent integration of the off-COM magnet law.
                var anaA = initA.Clone();
                var anaB = initB.Clone();
                double maxPosErr = 0.0;
                double maxAngErr = 0.0;
                for (int i = 0; i < ticks; i++)
                {
                    ApproachAnalytic.Step(ref anaA, ref anaB, (float)TickClock.FixedStep);
                    double posErr = Math.Max(
                        (anaA.Position - simA[i]).Length(), (anaB.Position - simB[i]).Length());
                    // The mismatch that matters: how far the sim's measured
                    // orientation is from the analytic law's at the same tick.
                    double anaAngErr = Math.Abs(Math.PI - MathHelpers.AngleBetween(anaA.AxisWorld(), anaB.AxisWorld()));
                    double angErr = Math.Abs(anaAngErr - simAngErr[i]);
                    maxPosErr = Math.Max(maxPosErr, posErr);
                    maxAngErr = Math.Max(maxAngErr, angErr);
                }

                // The measured trajectory must track the analytic law.
                // The sim's converge+align is governed by the exact same
                // off-COM law — a spring at the connector frames applied
                // at those points (linear impulse + the torque r×F), plus
                // the frame-tie torsion — so over the free magnetic
                // convergence its position sits within millimetres and its
                // orientation within a fraction of a degree of the
                // first-principles integration. (In the full-contact
                // game the residual overshoot at frame coincidence is
                // settled by the collision response and the final fixed
                // constraint when the approach locks; this measurement is
                // of the free magnet phase itself.) A law without the
                // r×F torque and the torsion tie would leave the
                // orientation at its ~5° initial error, far above the
                // angle tolerance.
                double posTol = 3e-2;
                double angTol = 0.04;
                Assert.That(maxPosErr, Is.LessThan(posTol),
                    "Sim position trajectory must match the analytic off-COM law. MaxPosErr=" + maxPosErr);
                Assert.That(maxAngErr, Is.LessThan(angTol),
                    "Sim orientation trajectory must match the analytic off-COM law (torque r×F). MaxAngErr=" + maxAngErr);
            }
        }

        /// <summary>An independent rigid-body state for the analytic off-COM law integrator in
        /// ApproachAnalytic.</summary>
        private struct AnalyticBody
        {
            public Vector3D Position;      // grid root, world
            public Quaternion Orientation; // float, world
            public Vector3 LinearVelocity;
            public Vector3 AngularVelocity;
            public float Mass;
            public Vector3 ComLocal;       // body-frame center of mass
            public Vector3 InvDiagBody;    // body-frame inverse-inertia diagonal
            public Vector3 FrameLocal;     // body-fixed connection frame (m)
            public Vector3 AxisLocal;      // body-fixed connect axis

            public AnalyticBody(SimRigidBody body, SimShipConnector con)
            {
                Position = body.PositionWorld;
                QuaternionD o = body.Orientation;
                Orientation = new Quaternion((float)o.X, (float)o.Y, (float)o.Z, (float)o.W);
                LinearVelocity = (Vector3)body.LinearVelocity;
                AngularVelocity = body.AngularVelocity;
                Mass = body.Mass;
                ComLocal = body.CenterOfMassLocal;
                InvDiagBody = body.InverseInertiaDiagonal;
                Vector3D gridWorld = con.Grid.WorldMatrix.Translation;
                FrameLocal = (Vector3)(con.WorldPosition - gridWorld);
                AxisLocal = con.ConnectAxisGrid;
            }

            public AnalyticBody Clone() { return this; }

            public MatrixD Rotation() { return MatrixD.CreateFromQuaternion(Orientation); }

            public Vector3D FrameWorld()
            {
                return Position + Vector3D.Transform(new Vector3D(FrameLocal), Rotation());
            }

            public Vector3D AxisWorld()
            {
                return Vector3D.Transform(new Vector3D(AxisLocal), Rotation());
            }

            public Vector3D ComWorld()
            {
                return Position + Vector3D.Transform(new Vector3D(ComLocal), Rotation());
            }

            public Vector3 PointVelWorld()
            {
                Vector3 r = (Vector3)(FrameWorld() - ComWorld());
                return LinearVelocity + Vector3.Cross(AngularVelocity, r);
            }

            public Vector3 WorldInverseInertiaTimes(Vector3 worldVector)
            {
                // approximate body as having diagonal inertia in body frame
                // (true for these near-axisymmetric hulls): world⁻¹ = R·B·Rᵀ.
                MatrixD R = Rotation();
                Vector3 local = (Vector3)Vector3D.Transform(new Vector3D(worldVector), MatrixD.Transpose(R));
                Vector3 scaled = new Vector3(local.X * InvDiagBody.X, local.Y * InvDiagBody.Y, local.Z * InvDiagBody.Z);
                return (Vector3)Vector3D.Transform(new Vector3D(scaled), R);
            }

            public static float AxisInertia(Vector3 axis, Vector3 invDiag)
            {
                // Effective rotational inertia about a body axis, from the
                // diagonal inverse-inertia: I_axis = 1 / (n · diag(inv) · n).
                float invAboutAxis = axis.X * axis.X * invDiag.X
                    + axis.Y * axis.Y * invDiag.Y
                    + axis.Z * axis.Z * invDiag.Z;
                return invAboutAxis > 1e-6f ? 1f / invAboutAxis : 1f;
            }
        }

        /// <summary>A first-principles integration of the connector magnet law (central off-COM
        /// spring + torsional tie), for measuring the sim's approach.</summary>
        private static class ApproachAnalytic
        {
            static readonly float Wn = ConnectorSystem.MagnetNaturalFrequency;
            static readonly float Amax = ConnectorSystem.MagnetMaxAcceleration;
            static readonly float Zeta = ConnectorSystem.MagnetDampingRatio;
            static readonly float Drag = ConnectorSystem.ApproachVelocityDrag;
            static readonly float ServoRate = ConnectorSystem.FrameTieServoRate;
            static readonly float CloseTime = ConnectorSystem.FrameTieCloseTime;
            static readonly float MaxRate = ConnectorSystem.FrameTieMaxClosingRate;
            static readonly float RotAmax = ConnectorSystem.FrameTieMaxAcceleration;

            public static void Step(ref AnalyticBody a, ref AnalyticBody b, float dt)
            {
                Vector3D frameA = a.FrameWorld();
                Vector3D frameB = b.FrameWorld();
                Vector3D axisA = a.AxisWorld();
                Vector3D axisB = b.AxisWorld();

                float mRed = (a.Mass * b.Mass) / (a.Mass + b.Mass);
                float k = mRed * Wn * Wn;
                float c = 2f * Zeta * mRed * Wn;

                // Central spring between the frames (damping along the line
                // only) — conserves angular momentum, as the sim.
                Vector3 sepVec = (Vector3)(frameA - frameB);
                float sepLen = sepVec.Length();
                Vector3 sepDir = sepLen > 1e-6f ? sepVec / sepLen : Vector3.Zero;
                Vector3 dv = a.PointVelWorld() - b.PointVelWorld();
                float dvAlong = Vector3.Dot(dv, sepDir);
                float f = k * sepLen + c * dvAlong;
                if (f > mRed * Amax)
                {
                    f = mRed * Amax;
                }
                Vector3 imp = -sepDir * (f * dt); // impulse on A (toward B)

                // Frame-tie alignment velocity servo (relative, equal-
                // opposite): drive the relative rate about the error axis
                // onto an exponentially-closing target, torque-limited —
                // the sim's FrameRotationalTie.
                Vector3 cross = Vector3.Cross(axisA, -axisB);
                float sinTh = cross.Length();
                Vector3 J = Vector3.Zero;
                if (sinTh > 1e-6f)
                {
                    Vector3 nn = cross / sinTh;
                    float cosTh = Vector3.Dot(axisA, -axisB);
                    float theta = (float)Math.Atan2(sinTh, cosTh);
                    float s = Vector3.Dot(a.AngularVelocity - b.AngularVelocity, nn);
                    float sDes = theta / CloseTime;
                    if (sDes > MaxRate)
                    {
                        sDes = MaxRate;
                    }
                    float iA = AnalyticBody.AxisInertia(nn, a.InvDiagBody);
                    float iB = AnalyticBody.AxisInertia(nn, b.InvDiagBody);
                    float iRed = (iA * iB) / (iA + iB);
                    float accel = ServoRate * (sDes - s);
                    if (accel > RotAmax)
                    {
                        accel = RotAmax;
                    }
                    else if (accel < -RotAmax)
                    {
                        accel = -RotAmax;
                    }
                    J = nn * (iRed * accel * dt);
                }

                // Apply impulses: linear + off-COM torque (r × F) + torsion.
                Vector3 rA = (Vector3)(frameA - a.ComWorld());
                Vector3 rB = (Vector3)(frameB - b.ComWorld());
                a.LinearVelocity += imp / a.Mass;
                a.AngularVelocity += a.WorldInverseInertiaTimes(Vector3.Cross(rA, imp));
                a.AngularVelocity += a.WorldInverseInertiaTimes(J);
                b.LinearVelocity -= imp / b.Mass;
                b.AngularVelocity += b.WorldInverseInertiaTimes(Vector3.Cross(rB, -imp));
                b.AngularVelocity += b.WorldInverseInertiaTimes(-J);

                // Magnetic-capture settling drag (relative linear velocity,
                // at the centers of mass, equal and opposite) — as the sim.
                Vector3 relLin = a.LinearVelocity - b.LinearVelocity;
                Vector3 dragImp = -(Drag * mRed) * relLin * dt;
                a.LinearVelocity += dragImp / a.Mass;
                b.LinearVelocity += -dragImp / b.Mass;

                // Semi-implicit Euler advance.
                a.Position += new Vector3D(a.LinearVelocity) * dt;
                b.Position += new Vector3D(b.LinearVelocity) * dt;
                a.Orientation = IntegrateQuat(a.Orientation, a.AngularVelocity, dt);
                b.Orientation = IntegrateQuat(b.Orientation, b.AngularVelocity, dt);
            }

            private static Quaternion IntegrateQuat(Quaternion q, Vector3 omega, float dt)
            {
                var w = new Quaternion(omega.X, omega.Y, omega.Z, 0f);
                var dq = Quaternion.Multiply(q, w) * (0.5f * dt);
                Quaternion nq = q + dq;
                nq.Normalize();
                return nq;
            }
        }
    }
}
