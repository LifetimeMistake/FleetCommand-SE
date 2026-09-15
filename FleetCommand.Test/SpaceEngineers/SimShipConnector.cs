using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using Havok;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// A ship connector, implementing the real
    /// <see cref="Sandbox.ModAPI.Ingame.IMyShipConnector"/> member by member
    /// (ground truth: Decompiled/Sandbox.Common/Sandbox/ModAPI/Ingame/IMyShipConnector.cs —
    /// <c>ThrowOut</c>, <c>CollectAll</c>, <c>PullStrength</c>, obsolete
    /// <c>IsLocked</c>, <c>IsConnected</c>, <c>Status</c>, <c>OtherConnector</c>,
    /// <c>IsParkingEnabled</c>, <c>Connect()</c>, <c>Disconnect()</c>,
    /// <c>ToggleConnect()</c>). Inherits the working-state + power stack from
    /// <see cref="SimFunctionalBlock"/>.
    /// <para>
    /// 's  boundary (design , ): the connector
    /// exposes a body-fixed <b>connection frame</b> — world pose (position +
    /// orientation) and point velocity derived from the owning rigid body —
    /// and supports docking: two unconnected connectors become
    /// <see cref="Status"/> <c>Connectable</c> when all configured capture
    /// gates pass (evaluated deterministically each tick by
    /// <see cref="SimWorld"/>; thresholds from
    /// <see cref="SePhysicsProfile"/>, ). <see cref="Connect()"/>
    /// succeeds only while connectable and creates a Havok fixed rigid
    /// constraint at the two connection frames (the game's
    /// <c>RecreateConstraintInternal</c>, MyShipConnector.cs:2030-2042); the
    /// two grids then move as one. <see cref="Disconnect()"/> removes it and
    /// returns both connectors to a clean state. <see cref="Status"/> follows
    /// the game mapping (MyShipConnector.cs:599-611): Connected when
    /// connected; else Connectable when working AND the capture gates pass
    /// (the game's <c>IsWorking &amp;&amp; InConstraint</c>); else Unconnected.
    /// No magnetic attraction / inventory / trading / conveyor modeling.
    /// </para>
    /// </summary>
    public class SimShipConnector : SimFunctionalBlock, IMyShipConnector
    {
        /// <summary>Local forward axis of the connection frame (VRageMath: Forward = (0, 0, -1)).</summary>
        private static readonly Vector3 LocalForward = new Vector3(0f, 0f, -1f);

        private readonly Matrix _blockRotation;
        private readonly Matrix _connectorRotation;
        private readonly float _blockReach;
        private readonly float _faceDepth;
        private readonly bool _throwOut;
        private readonly bool _collectAll;
        private readonly float _pullStrength;
        private bool _parkingEnabled = true;

        // Connection state, mirroring the game's master/slave pairing
        // (MyShipConnector.IsMaster — block with higher EntityId is master).
        private bool _connected;
        private bool _captured;            // world capture phase result: an eligible unconnected partner passed all gates
        private SimShipConnector _capturedPartner;
        private bool _isMaster;
        private SimShipConnector _partner;
        private HkConstraint _constraint;  // valid on the master while connected

        // .5 approach state (the game's two-stage connect): between
        // "connectable" (capture) and the final rigid lock, Connect() begins
        // the approach — the two free grids are pulled together and their
        // connect axes aligned (matching CreateConstraint's magnetic
        // convergence), and only on meet is the final fixed constraint
        // created (Connected). While approaching, Status stays Connectable
        // (the game's InConstraint); the master (higher EntityId) is the
        // approach driver (the game's GetMaster).
        private bool _approaching;
        private SimShipConnector _approachMaster;
        private SimShipConnector _approachPartner;

        public SimShipConnector(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            _blockRotation = Matrix.CreateFromQuaternion(template.Orientation);
            _connectorRotation = Matrix.CreateFromQuaternion(template.ConnectorOrientation);
            _blockReach = template.ConnectorReach;
            // The connection point sits at the connector's mating face, not
            // the block centre: offset from the block centre along the
            // connect axis by the block's half-extent. Two connectors
            // lock when these face points coincide (the dock plane), so the
            // approach converges the leading faces to contact without the
            // two hulls interpenetrating (matching the game's connect
            // frame at the connector's outer face).
            _faceDepth = Math.Max(template.HalfExtents.X,
                Math.Max(template.HalfExtents.Y, template.HalfExtents.Z));
            _throwOut = false;
            _collectAll = false;
            _pullStrength = 0f;
        }

        // — connection frame (derived from the owning rigid body) —

        /// <summary>
        /// World position of the connection point (the body-fixed
        /// connection frame's origin): the block's world position plus an
        /// offset along the connect axis to the connector's mating face,
        /// derived from the owning grid's rigid body plus the block local
        /// pose (the game's <c>ConstraintPositionWorld</c>, MyShipConnector.cs:1939).
        /// </summary>
        public Vector3D WorldPosition
        {
            get
            {
                Vector3D center = base.WorldMatrix.Translation;
                return center + ConnectAxisWorld * _faceDepth;
            }
        }

        /// <summary>
        /// World orientation of the connection frame: the grid body's
        /// rotation applied to the block's layout rotation and the
        /// connector's own <see cref="TemplateBlock.ConnectorOrientation"/>.
        /// </summary>
        public MatrixD WorldOrientation
        {
            get
            {
                Matrix gridRot = ((Matrix)Grid.WorldMatrix).GetOrientation();
                return (MatrixD)(gridRot * _blockRotation * _connectorRotation);
            }
        }

        /// <summary>
        /// The connect axis in world space: the direction the connector
        /// extends to meet a partner (the connection frame's local forward
        /// — the body-fixed face). Two connectors are aligned for docking
        /// when their connect axes are (nearly) opposing.
        /// </summary>
        public Vector3D ConnectAxisWorld
        {
            get
            {
                Vector3 axis = Vector3.TransformNormal(LocalForward, (Matrix)WorldOrientation);
                return new Vector3D(axis);
            }
        }

        /// <summary>
        /// The connect axis in grid-body coordinates (the block's local
        /// forward transformed by the layout and connector rotations).
        /// </summary>
        public Vector3 ConnectAxisGrid
        {
            get
            {
                return Vector3.TransformNormal(Vector3.TransformNormal(LocalForward, _blockRotation), _connectorRotation);
            }
        }

        /// <summary>
        /// World linear velocity of the connection point: the body's linear
        /// velocity plus the rotational contribution, <c>v + ω × r</c>,
        /// where r is the connection point relative to the grid's center of
        /// mass ( "connector-point velocities"). Derived from the
        /// owning rigid body.
        /// </summary>
        public Vector3D PointVelocityWorld
        {
            get
            {
                SimRigidBody body = Grid.Body;
                MatrixD world = Grid.WorldMatrix;
                Vector3D comWorld = Vector3D.Transform(new Vector3D(body.CenterOfMassLocal), world);
                Vector3 omega = body.AngularVelocity;
                Vector3 r = (Vector3)(WorldPosition - comWorld);
                return body.LinearVelocity + new Vector3D(Vector3.Cross(omega, r));
            }
        }

        /// <summary>
        /// This connector's effective capture reach: its own
        /// <see cref="TemplateBlock.ConnectorReach"/> when set, else the
        /// physics profile's <c>ConnectorReach</c> ().
        /// </summary>
        public float EffectiveReach => _blockReach > 0f ? _blockReach : Grid.World.Profile.ConnectorReach;

        // — connection state (sim surface for tests and the world capture) —

        /// <summary>True while this connector is docked to a partner.</summary>
        internal bool ConnectedInternal => _connected;

        /// <summary>The partner currently docked to this connector, or null.</summary>
        internal SimShipConnector PartnerInternal => _partner;

        /// <summary>True when the world capture phase found an eligible unconnected partner for this connector.</summary>
        internal bool Captured => _captured;

        /// <summary>True while this connector is in the approach (pre-lock) stage.</summary>
        internal bool ApproachingInternal => _approaching;

        /// <summary>The approach master (higher EntityId) for this connector, or null.</summary>
        internal SimShipConnector ApproachMasterInternal => _approachMaster;

        /// <summary>The approach partner for the master, or null.</summary>
        internal SimShipConnector ApproachPartnerInternal => _approachPartner;

        /// <summary>Sets this connector's approach role; called by <see cref="ConnectorSystem.BeginApproach"/>.</summary>
        internal void SetApproach(SimShipConnector master, SimShipConnector partner)
        {
            _approaching = true;
            _approachMaster = master;
            _approachPartner = partner;
        }

        /// <summary>Clears this connector's approach role (idempotent).</summary>
        internal void ClearApproach()
        {
            _approaching = false;
            _approachMaster = null;
            _approachPartner = null;
        }

        /// <summary>Resets this connector's per-tick capture result.</summary>
        internal void ResetCapture()
        {
            _captured = false;
            _capturedPartner = null;
        }

        /// <summary>Marks this connector as captured toward <paramref name="partner"/>.</summary>
        internal void MarkCaptured(SimShipConnector partner)
        {
            _captured = true;
            _capturedPartner = partner;
        }

        // — IMyShipConnector —

        /// <inheritdoc />
        public bool ThrowOut
        {
            get { return _throwOut; }
            set { throw ThrowSet("IMyShipConnector.ThrowOut"); }
        }

        /// <inheritdoc />
        public bool CollectAll
        {
            get { return _collectAll; }
            set { throw ThrowSet("IMyShipConnector.CollectAll"); }
        }

        /// <inheritdoc />
        public float PullStrength
        {
            get { return _pullStrength; }
            set { throw ThrowSet("IMyShipConnector.PullStrength"); }
        }

        /// <inheritdoc />
        [Obsolete("Use the Status property.")]
        public bool IsLocked => Status != MyShipConnectorStatus.Unconnected;

        /// <inheritdoc />
        public bool IsConnected => _connected;

        /// <inheritdoc />
        public MyShipConnectorStatus Status
        {
            get
            {
                if (_connected)
                {
                    return MyShipConnectorStatus.Connected;
                }
                // Connectable while approaching (the game's InConstraint,
                // MyShipConnector.cs:599-611) OR while the world's capture
                // gates pass; IsWorking gates both.
                if (IsWorking && (_approaching || _captured))
                {
                    return MyShipConnectorStatus.Connectable;
                }
                return MyShipConnectorStatus.Unconnected;
            }
        }

        /// <inheritdoc />
        public IMyShipConnector OtherConnector => _partner;

        /// <inheritdoc />
        /// <summary>A plain stored value (default true), parity with the
        /// game which merely stores it and never uses it in connect logic
        /// (MyShipConnector.cs:436-446; representation-only).</summary>
        public bool IsParkingEnabled
        {
            get { return _parkingEnabled; }
            set { _parkingEnabled = value; }
        }

        /// <inheritdoc />
        public void Connect()
        {
            // The interface contract: does nothing unless Status is
            // Connectable (MyShipConnector.cs TryConnect gates on
            // InConstraint && !Connected; the interface doc: "If Status is
            // anything else but Connectable, this method does nothing").
            if (Status != MyShipConnectorStatus.Connectable)
            {
                return;
            }
            SimShipConnector other = _capturedPartner;
            if (other == null)
            {
                return;
            }
            // Two-stage connect (.5): a connectable-but-not-yet-
            // converged pair begins the approach (the game's CreateConstraint
            // magnetic convergence); the pair only becomes Connected once
            // the connectors physically meet and the axes are aligned.
            ConnectorSystem.BeginApproach(this, other);
        }

        /// <inheritdoc />
        public void Disconnect()
        {
            if (_connected)
            {
                SimShipConnector other = _partner;
                SimShipConnector master = (_isMaster || other == null) ? this : other;
                master.RemoveConstraint();
                ClearConnection();
                if (other != null)
                {
                    other.ClearConnection();
                }
                return;
            }
            if (_approaching)
            {
                // Release a mid-approach pair: both return to a clean free
                // state — no residual constraint AND no stale capture
                // result, so Status is Unconnected again until the world
                // re-captures them.
                SimShipConnector partner = _approachPartner ?? _approachMaster;
                ConnectorSystem.CancelApproach(this);
                ResetCapture();
                if (partner != null && !ReferenceEquals(partner, this))
                {
                    partner.ResetCapture();
                }
            }
        }

        /// <inheritdoc />
        public void ToggleConnect()
        {
            if (_connected)
            {
                Disconnect();
            }
            else
            {
                Connect();
            }
        }

        // — Havok constraint —

        /// <summary>
        /// Creates a Havok fixed rigid constraint tying this body to
        /// <paramref name="slave"/>'s body at the two connection frames
        /// (mirroring the game's <c>RecreateConstraintInternal</c>,
        /// MyShipConnector.cs:2030-2042) and adds it to the world.
        /// </summary>
        private void CreateConstraint(SimShipConnector slave)
        {
            MatrixD frame = ConnectionFrameWorld;
            Matrix pivotA = (Matrix)(frame * MatrixD.Invert(Grid.WorldMatrix));
            Matrix pivotB = (Matrix)(frame * MatrixD.Invert(slave.Grid.WorldMatrix));

            var data = new HkFixedConstraintData();
            data.SetInBodySpaceInternal(ref pivotA, ref pivotB);
            data.SetSolvingMethod(HkSolvingMethod.MethodStabilized);

            _constraint = new HkConstraint(Grid.Body.Body, slave.Grid.Body.Body, data);
            Grid.World.AddConstraint(_constraint);
        }

        /// <summary>Removes and disposes the constraint and clears the master's connection state.</summary>
        private void RemoveConstraint()
        {
            if (_constraint != null)
            {
                Grid.World.RemoveConstraint(_constraint);
                _constraint.Dispose();
                _constraint = null;
            }
        }

        /// <summary>
        /// Clears this connector's connection state (idempotent). Also
        /// clears any approach role.
        /// </summary>
        private void ClearConnection()
        {
            _constraint = null;
            _connected = false;
            _isMaster = false;
            _partner = null;
            _captured = false;
            _capturedPartner = null;
            ClearApproach();
        }

        /// <summary>
        /// The final lock of an approach pair: this connector is the master
        /// (higher EntityId, per <c>MyShipConnector.GetMaster</c>), creates
        /// the Havok fixed rigid constraint (the game's
        /// <c>RecreateConstraintInternal</c>, :2030-2042), and marks both
        /// connectors Connected. Called by <see cref="ConnectorSystem"/>
        /// once the approach has converged.
        /// </summary>
        internal void LockTo(SimShipConnector slave)
        {
            _isMaster = true;
            slave._isMaster = false;
            CreateConstraint(slave);
            _connected = true;
            slave._connected = true;
            _partner = slave;
            slave._partner = this;
            _captured = false;
            _capturedPartner = null;
            ClearApproach();
            slave.ClearApproach();
        }

        private MatrixD ConnectionFrameWorld
        {
            get
            {
                MatrixD m = WorldOrientation;
                m.Translation = WorldPosition;
                return m;
            }
        }

        /// <summary>Throws for a property setter, naming <paramref name="member"/>.</summary>
        private static NotSupportedException ThrowSet(string member)
        {
            return new NotSupportedException(
                "SimShipConnector does not implement '" + member + "' (stored scenario state only — no magnetic/inventory model, ).");
        }
    }
}
