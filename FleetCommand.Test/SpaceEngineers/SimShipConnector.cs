using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using Havok;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A ship connector implementing <see cref="IMyShipConnector"/> member by member
    /// (ground truth: Decompiled/Sandbox.Common/Sandbox/ModAPI/Ingame/IMyShipConnector.cs —
    /// ThrowOut, CollectAll, PullStrength, obsolete IsLocked, IsConnected, Status, OtherConnector,
    /// IsParkingEnabled, Connect(), Disconnect(), ToggleConnect()); inherits the working-state +
    /// power stack from <see cref="SimFunctionalBlock"/>. Exposes a body-fixed connection frame
    /// (world pose and point velocity derived from the owning rigid body) and reproduces the
    /// game's full connector lifecycle, driven each tick by <see cref="ConnectorSystem"/></summary>
    public class SimShipConnector : SimFunctionalBlock, IMyShipConnector
    {
        /// <summary>Local forward axis of the connection frame (VRageMath: Forward = (0, 0,
        /// -1)).</summary>
        private static readonly Vector3 LocalForward = new Vector3(0f, 0f, -1f);

        private readonly Matrix _blockRotation;
        private readonly Matrix _connectorRotation;
        private readonly float _faceDepth;
        private readonly bool _throwOut;
        private readonly bool _collectAll;

        /// <summary>The connector's pull strength in [0, 1] (the game's <c>Strength</c> slider,
        /// MyShipConnector.cs:845-869); the effective (min of the pair) drives the malleable
        /// hinge's hold. Template-default 1.</summary>
        private readonly float _strength;

        private bool _parkingEnabled = true;

        // Connection state, mirroring the game's placement (MyShipConnector): a connector is
        // IN a constraint (paired with a partner by the malleable hinge — the approach) before
        // it is Connected (welded by the fixed constraint).
        private bool _inConstraint;
        private bool _connected;
        private bool _isMaster;
        private SimShipConnector _partner;
        private HkConstraint _constraint;  // valid on the master; malleable while approaching, fixed when connected

        public SimShipConnector(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
            _blockRotation = Matrix.CreateFromQuaternion(template.Orientation);
            _connectorRotation = Matrix.CreateFromQuaternion(template.ConnectorOrientation);
            // The connection point sits at the connector's mating face, not the block centre:
            // offset from the block centre along the connect axis by the block's half-extent.
            // This is the game's detector radius too — m_detectorRadius = halfExtents.AbsMax()
            // (MyShipConnector.cs:1280), the connector's own block size.
            _faceDepth = Math.Max(template.HalfExtents.X,
                Math.Max(template.HalfExtents.Y, template.HalfExtents.Z));
            _throwOut = false;
            _collectAll = false;
            _strength = template.ConnectorStrength;
        }

        // — connection frame (derived from the owning rigid body) —

        /// <summary>World position of the connection point (the body-fixed connection frame's
        /// origin): the block's world position plus an offset along the connect axis to the
        /// connector's mating face, derived from the owning grid's rigid body plus the block local
        /// pose (the game's <c>ConstraintPositionWorld</c>, MyShipConnector.cs:1939).</summary>
        public Vector3D WorldPosition
        {
            get
            {
                Vector3D center = base.WorldMatrix.Translation;
                return center + ConnectAxisWorld * _faceDepth;
            }
        }

        /// <summary>World orientation of the connection frame: the grid body's rotation applied to
        /// the block's layout rotation and the connector's own
        /// <see cref="TemplateBlock.ConnectorOrientation"/>.</summary>
        public MatrixD WorldOrientation
        {
            get
            {
                Matrix gridRot = ((Matrix)Grid.WorldMatrix).GetOrientation();
                return (MatrixD)(gridRot * _blockRotation * _connectorRotation);
            }
        }

        /// <summary>The connect axis in world space: the direction the connector extends to meet a
        /// partner (the connection frame's local forward — the body-fixed face). Two connectors are
        /// aligned for docking when their connect axes are (nearly) opposing.</summary>
        public Vector3D ConnectAxisWorld
        {
            get
            {
                Vector3 axis = Vector3.TransformNormal(LocalForward, (Matrix)WorldOrientation);
                return new Vector3D(axis);
            }
        }

        /// <summary>The connect axis in grid-body coordinates (the block's local forward
        /// transformed by the layout and connector rotations) — the game's
        /// <c>ConstraintAxisGridSpace</c> (MyShipConnector.cs:1954).</summary>
        public Vector3 ConnectAxisGrid
        {
            get
            {
                return Vector3.TransformNormal(Vector3.TransformNormal(LocalForward, _blockRotation), _connectorRotation);
            }
        }

        /// <summary>World linear velocity of the connection point: the body's linear velocity plus
        /// the rotational contribution, v + ω × r, where r is the connection point relative to the
        /// grid's center of mass. Derived from the owning rigid body.</summary>
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

        /// <summary>This connector's auto-detect sphere radius: its own block half-extent max
        /// (the game's <c>m_detectorRadius = halfExtents.AbsMax()</c>, MyShipConnector.cs:1280).
        /// The broadphase that FindOtherConnector searches within.</summary>
        public float DetectorRadius => _faceDepth;

        /// <summary>The grid-local (origin-relative) position of the connection point — the
        /// game's <c>ConstraintPositionInGridSpace</c> relative frame (MyShipConnector.cs:1944),
        /// the body-space convention the Havok hinge is authored in.</summary>
        private Vector3 GridLocalConnectionPosition
        {
            get
            {
                MatrixD world = Grid.WorldMatrix;
                MatrixD inverse;
                MatrixD.Invert(ref world, out inverse);
                return (Vector3)Vector3D.Transform(WorldPosition, inverse);
            }
        }

        /// <summary>The connector's pull strength (0..1), pair-min used as the approach-hold
        /// effectiveness.</summary>
        public float Strength => _strength;

        // — connection state (sim surface for tests and the world capture) —

        /// <summary>True while this connector is paired with a partner — either approaching via
        /// the malleable hinge or Connected via the fixed constraint (the game's
        /// <c>InConstraint</c>).</summary>
        internal bool InConstraint => _inConstraint;

        /// <summary>True while this connector is docked to a partner (welded).</summary>
        internal bool ConnectedInternal => _connected;

        /// <summary>True while this connector is in the approach stage (paired, not yet
        /// welded).</summary>
        internal bool ApproachingInternal => _inConstraint && !_connected;

        /// <summary>True when this connector is the pair's master (higher EntityId), the one the
        /// game has create the shared constraint.</summary>
        internal bool IsMaster => _isMaster;

        /// <summary>The partner currently paired with this connector, or null.</summary>
        internal SimShipConnector PartnerInternal => _partner;

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
        /// <summary>Reads the connector's Strength (the game's IMyShipConnector.PullStrength
        /// getter returns Strength, MyShipConnector.cs:557-562). Writing is out-of-contract in
        /// this framework's stored scenario state (Strength comes from the template).</summary>
        public float PullStrength
        {
            get { return _strength; }
            set { throw ThrowSet("IMyShipConnector.PullStrength"); }
        }

        /// <inheritdoc />
        [Obsolete("Use the Status property.")]
        public bool IsLocked => IsWorking && InConstraint;

        /// <inheritdoc />
        public bool IsConnected => _connected;

        /// <inheritdoc />
        /// <summary>The game's Status mapping (MyShipConnector.cs:599-613): Connected when
        /// welded; else Connectable when IsWorking AND InConstraint (paired, approaching); else
        /// Unconnected.</summary>
        public MyShipConnectorStatus Status
        {
            get
            {
                if (_connected)
                {
                    return MyShipConnectorStatus.Connected;
                }
                if (IsWorking && _inConstraint)
                {
                    return MyShipConnectorStatus.Connectable;
                }
                return MyShipConnectorStatus.Unconnected;
            }
        }

        /// <inheritdoc />
        public IMyShipConnector OtherConnector => _partner;

        /// <inheritdoc />
        /// <summary>A plain stored value (default true), parity with the game which merely stores
        /// it and never uses it in connect logic (MyShipConnector.cs:436-446;
        /// representation-only).</summary>
        public bool IsParkingEnabled
        {
            get { return _parkingEnabled; }
            set { _parkingEnabled = value; }
        }

        /// <inheritdoc />
        /// <summary>Welds an approaching (paired, not-yet-connected) pair — the game's
        /// <c>TryConnect</c> (:919-936), which returns immediately unless <c>InConstraint &amp;&amp;
        /// !Connected</c>, then calls <c>Weld()</c>.</summary>
        public void Connect()
        {
            if (InConstraint && !ConnectedInternal)
            {
                SimShipConnector master = _isMaster ? this : _partner;
                if (master != null)
                {
                    master.WeldPair();
                }
            }
        }

        /// <inheritdoc />
        /// <summary>Removes the link and returns both to a free state — the game's
        /// <c>TryDisconnect</c> (:951-974) → <c>Detach</c>, gated on
        /// <c>InConstraint &amp;&amp; Connected</c>.</summary>
        public void Disconnect()
        {
            if (InConstraint && ConnectedInternal)
            {
                SimShipConnector master = _isMaster ? this : _partner;
                if (master != null)
                {
                    master.DetachFromMaster();
                }
            }
        }

        /// <inheritdoc />
        /// <summary>The game's <c>TrySwitch</c> (:901-914): toggles a paired pair between
        /// Connect and Disconnect; unpaired pairs do nothing.</summary>
        public void ToggleConnect()
        {
            if (InConstraint)
            {
                if (ConnectedInternal)
                {
                    Disconnect();
                }
                else
                {
                    Connect();
                }
            }
        }

        // — game lifecycle (driven by ConnectorSystem each tick) —

        /// <summary>Detaches this connector's link (if any), routing to the master — the game's
        /// <c>Detach</c> (:2158-2222), which the master performs. Used when the pair stops
        /// working or drifts beyond the keep range.</summary>
        internal void DetachLink()
        {
            if (!_inConstraint)
            {
                return;
            }
            SimShipConnector master = _isMaster ? this : _partner;
            if (master != null && master._inConstraint)
            {
                master.DetachFromMaster();
            }
        }

        /// <summary>Attaches this connector (as master) to <paramref name="other"/> via the
        /// game's malleable (hinge) approach constraint — <c>TryAttach</c> → <c>GetMaster</c> →
        /// <c>CreateConstraint</c>, MyShipConnector.cs:1494-1505, CreateConstraintNosync
        /// :1906-1923: a hinge wrapped in an <see cref="HkMalleableConstraintData"/> with the
        /// effective strength; the pair is now InConstraint and not Connected, and Havok's solver
        /// drives the convergence (the game writes no hand-written spring).</summary>
        internal void AttachAsMaster(SimShipConnector other)
        {
            // The hinge's two anchors and axes, authored body-space (grid-local) — the game's
            // ConstraintPositionInGridSpace / ConstraintAxisGridSpace, with the partner's axis
            // negated so both point along the shared line (:1908-1913).
            Vector3 posA = GridLocalConnectionPosition;
            Vector3 axisA = Vector3.Normalize(ConnectAxisGrid);
            Vector3 posB = other.GridLocalConnectionPosition;
            Vector3 axisB = Vector3.Normalize(-other.ConnectAxisGrid);

            var hinge = new HkHingeConstraintData();
            hinge.SetInBodySpaceInternal(ref posA, ref posB, ref axisA, ref axisB);
            var malleable = new HkMalleableConstraintData();
            malleable.SetData(hinge);
            hinge.ClearHandle();
            hinge = null;
            malleable.Strength = EffectiveStrength(other);

            _constraint = new HkConstraint(Grid.Body.Body, other.Grid.Body.Body, malleable);
            Grid.World.AddConstraint(_constraint);

            _isMaster = true;
            _inConstraint = true;
            _connected = false;
            _partner = other;
            other._isMaster = false;
            other._inConstraint = true;
            other._connected = false;
            other._partner = this;
        }

        /// <summary>Refreshes the approach constraint's malleable strength to the pair's
        /// effective value — the game's per-tick approach-keep (:1559-1569), driven by the
        /// master.</summary>
        internal void UpdateApproachStrength()
        {
            if (_constraint != null && _constraint.ConstraintData is HkMalleableConstraintData malleable)
            {
                malleable.Strength = EffectiveStrength(_partner);
            }
        }

        /// <summary>Welds this connector's pair: routes to the master (which owns the shared
        /// constraint), which removes the malleable (hinge) approach constraint and replaces it
        /// with the fixed constraint, marking both Connected — the game's <c>Weld</c> →
        /// <c>RecreateConstraintInternal</c> (MyShipConnector.cs:1969, :2022-2042). The master
        /// drives this so the auto-weld in the approach branch never leaves the malleable behind.
        /// </summary>
        internal void WeldPair()
        {
            SimShipConnector master = _isMaster ? this : _partner;
            if (master != null && master.InConstraint && !master.ConnectedInternal)
            {
                master.DoWeld();
            }
        }

        private void DoWeld()
        {
            SimShipConnector other = _partner;
            if (other == null)
            {
                return;
            }
            RemoveConstraint();
            CreateConstraint(other);
            _connected = true;
            other._connected = true;
        }

        /// <summary>The master removes the shared constraint and returns both connectors to a
        /// clean, free state — the game's <c>DetachInternal</c> (MyShipConnector.cs:2224-2268).
        /// </summary>
        internal void DetachFromMaster()
        {
            SimShipConnector other = _partner;
            RemoveConstraint();
            ClearLink();
            if (other != null && !ReferenceEquals(other, this))
            {
                other.ClearLink();
            }
        }

        private void ClearLink()
        {
            _constraint = null;
            _inConstraint = false;
            _connected = false;
            _isMaster = false;
            _partner = null;
        }

        /// <summary>The effective pull strength for this pair — the min of the two connector's
        /// Strength, floored at the game's 1e-6 log limit (MyShipConnector.cs:1000-1012,
        /// GetEffectiveStrength).</summary>
        private float EffectiveStrength(SimShipConnector other)
        {
            float s = Math.Min(_strength, other._strength);
            return s < 1e-6f ? 1e-6f : s;
        }

        // — Havok constraint —

        /// <summary>Creates a Havok fixed rigid constraint tying this body to
        /// <paramref name="slave"/>'s body at the two connection frames (mirroring the game's
        /// <c>RecreateConstraintInternal</c>, MyShipConnector.cs:2030-2042) and adds it to the
        /// world.</summary>
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

        /// <summary>Removes and disposes the constraint and clears the master's reference to
        /// it.</summary>
        private void RemoveConstraint()
        {
            if (_constraint != null)
            {
                Grid.World.RemoveConstraint(_constraint);
                _constraint.Dispose();
                _constraint = null;
            }
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
                "SimShipConnector does not implement '" + member + "' (stored scenario state only — no magnetic/inventory model).");
        }
    }
}
