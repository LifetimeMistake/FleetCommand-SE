using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The connector docking capture logic (design , ): the
    /// deterministic, configurable capture gates that decide when two
    /// unconnected connectors become <c>Connectable</c>. The game's live
    /// docking search is a complex physics proximity scan; the framework
    /// models capture as four discrete gates (separation, opposing-axis
    /// angular error, relative linear velocity at the connector points,
    /// relative angular velocity) per , with thresholds from the
    /// world's <see cref="SePhysicsProfile"/>, so docking tests can
    /// distinguish approach convergence, capture readiness, and an actual
    /// connection deterministically. Driven each tick by
    /// <see cref="SimWorld"/> in phase 5; the <see cref="SimShipConnector"/>
    /// stores the mutually-captured result (capture always pairs
    /// peer connectors so an eligible pair is <em>both</em> Connectable).
    /// </summary>
    internal static class ConnectorSystem
    {
        /// <summary>
        /// Evaluates the capture gates over all unconnected connectors in
        /// deterministic order, marking eligible unconnected pairs as
        /// mutually captured. Connectors on the same grid are never docking
        /// partners (the framework models cross-grid docking).
        /// </summary>
        public static void EvaluateCapture(IReadOnlyList<SimShipConnector> connectors, SePhysicsProfile profile)
        {
            if (connectors == null)
            {
                throw new ArgumentNullException(nameof(connectors));
            }
            if (profile == null)
            {
                throw new ArgumentNullException(nameof(profile));
            }

            for (int i = 0; i < connectors.Count; i++)
            {
                connectors[i].ResetCapture();
            }
            for (int i = 0; i < connectors.Count; i++)
            {
                SimShipConnector a = connectors[i];
                if (a.Closed || a.ConnectedInternal || !a.IsWorking)
                {
                    continue;
                }
                for (int j = i + 1; j < connectors.Count; j++)
                {
                    SimShipConnector b = connectors[j];
                    if (b.Closed || b.ConnectedInternal || !b.IsWorking)
                    {
                        continue;
                    }
                    if (ReferenceEquals(a.Grid, b.Grid))
                    {
                        continue;
                    }
                    if (CapturePasses(a, b, profile))
                    {
                        a.MarkCaptured(b);
                        b.MarkCaptured(a);
                    }
                }
            }
        }

        /// <summary>
        /// The four  capture gates. A pair passes when all of:
        /// separation ≤ min of the two connectors' effective reach and the
        /// profile reach; opposing-axis angular error ≤
        /// <c>profile.ConnectorAngularErrorMax</c>; relative linear
        /// velocity at the connector points ≤
        /// <c>profile.ConnectorLinearVelocityMax</c>; relative angular
        /// velocity ≤ <c>profile.ConnectorAngularVelocityMax</c>.
        /// </summary>
        public static bool CapturePasses(SimShipConnector a, SimShipConnector b, SePhysicsProfile profile)
        {
            // 1. Separation gate.
            float separation = (float)(a.WorldPosition - b.WorldPosition).Length();
            float reach = Math.Min(profile.ConnectorReach, Math.Min(a.EffectiveReach, b.EffectiveReach));
            if (separation > reach)
            {
                return false;
            }

            // 2. Opposing-axis angular error gate: how far the two connect
            // axes deviate from exactly opposing (error = |π - angle|).
            double angle = AngleBetween(a.ConnectAxisWorld, b.ConnectAxisWorld);
            double angularError = Math.Abs(Math.PI - angle);
            if (angularError > profile.ConnectorAngularErrorMax)
            {
                return false;
            }

            // 3. Relative linear velocity at the connector points gate.
            double relativeVelocity = (a.PointVelocityWorld - b.PointVelocityWorld).Length();
            if (relativeVelocity > profile.ConnectorLinearVelocityMax)
            {
                return false;
            }

            // 4. Relative angular velocity gate.
            double relativeAngularVelocity = (a.Grid.Body.AngularVelocity - b.Grid.Body.AngularVelocity).Length();
            if (relativeAngularVelocity > profile.ConnectorAngularVelocityMax)
            {
                return false;
            }

            return true;
        }

        private static double AngleBetween(Vector3D a, Vector3D b)
        {
            double dot = Vector3D.Dot(Vector3D.Normalize(a), Vector3D.Normalize(b));
            dot = dot < -1.0 ? -1.0 : dot > 1.0 ? 1.0 : dot;
            return Math.Acos(dot);
        }

        // — .5: the two-stage connect (approach then lock) —

        // Approach convergence (deterministic, fixed 1/60 s step): a
        // physical, critically-damped spring between the two connector
        // points (force at each point via the reduced mass, natural
        // frequency wn, with critical damping c = 2 mr wn). Applied at the
        // connector points, the pull both translates the bodies together
        // and (being off-COM) rotates them to align, as the game's
        // CreateConstraint magnet does. A secondary gentle servo aligns the
        // connect axes to anti-parallel for the lock gate.
        private const float ApproachWn = 5f;     // spring natural frequency (rad/s)
        private const float ApproachAmax = 30f;  // max pull acceleration (m/s^2)
        private const float ApproachAngularGain = 10f;  // rad/s per rad of axis error
        private const float ApproachAngularStep = 1f;
        private const float ApproachAngularCap = 4f;    // rad/s

        /// <summary>
        /// Starts the approach for a connectable pair (the game's
        /// <c>TryAttach</c>→<c>CreateConstraint</c>, MyShipConnector.cs:1478-1520,
        /// :1882): the master (higher EntityId, the game's <c>GetMaster</c>)
        /// drives the convergence. If the pair is already effectively
        /// touching and aligned, it locks immediately instead.
        /// </summary>
        public static void BeginApproach(SimShipConnector a, SimShipConnector b)
        {
            SimShipConnector master = a.Grid.EntityId >= b.Grid.EntityId ? a : b;
            SimShipConnector slave = ReferenceEquals(master, a) ? b : a;
            if (Converged(master, slave, master.Grid.World.Profile))
            {
                master.LockTo(slave);
                return;
            }
            master.SetApproach(master, slave);
            slave.SetApproach(master, null);
        }

        /// <summary>
        /// Cancels an in-progress approach (release): both connectors return
        /// to a clean, unconnected, free state (no residual constraint).
        /// </summary>
        public static void CancelApproach(SimShipConnector c)
        {
            SimShipConnector master = (c.ApproachMasterInternal != null && !ReferenceEquals(c.ApproachMasterInternal, c))
                ? c.ApproachMasterInternal : c;
            SimShipConnector partner = ReferenceEquals(master, c) ? c.ApproachPartnerInternal : c;
            if (master != null)
            {
                master.ClearApproach();
            }
            if (partner != null && !ReferenceEquals(partner, master))
            {
                partner.ClearApproach();
            }
        }

        /// <summary>
        /// Tick phase 6: apply the approach convergence — a pull on each
        /// body at its connector point driving the connector points to
        /// coincidence, plus an alignment torque rotating each grid's
        /// connect axis to anti-parallel — matching the game's
        /// <c>CreateConstraint</c> magnet (a hinge at the connector frames
        /// with finite strength). Only the master drives; deterministic.
        /// </summary>
        public static void ApplyApproach(IReadOnlyList<SimShipConnector> connectors)
        {
            if (connectors == null)
            {
                return;
            }
            for (int i = 0; i < connectors.Count; i++)
            {
                SimShipConnector c = connectors[i];
                if (c.Closed || !c.ApproachingInternal)
                {
                    continue;
                }
                if (!ReferenceEquals(c.ApproachMasterInternal, c))
                {
                    continue;
                }
                SimShipConnector slave = c.ApproachPartnerInternal;
                if (slave == null || slave.Closed)
                {
                    continue;
                }
                ApplyConvergence(c, slave);
            }
        }

        private static void ApplyConvergence(SimShipConnector a, SimShipConnector b)
        {
            SimRigidBody ba = a.Grid.Body;
            SimRigidBody bb = b.Grid.Body;
            if (ba == null || bb == null || ba.IsDisposed || bb.IsDisposed || !ba.IsDynamic || !bb.IsDynamic)
            {
                return;
            }

            Vector3D rA = a.WorldPosition;
            Vector3D rB = b.WorldPosition;
            Vector3D axisA = a.ConnectAxisWorld;
            Vector3D axisB = b.ConnectAxisWorld;

            // The spring between the connector points, applied at each
            // body's center of mass: a critically-damped spring on the
            // reduced mass that pulls the bodies together in pure
            // translation (no off-COM torque), while a separate stiff
            // alignment servo below rotates the connect axes anti-parallel.
            // Decoupling translation from rotation keeps the convergence
            // stable as the heavy hulls close — the magnetic pull drives
            // the connect points together, the alignment brings the faces
            // to anti-parallel, and only on meet does the pair lock.
            float mRed = (ba.Mass * bb.Mass) / (ba.Mass + bb.Mass);
            float k = mRed * ApproachWn * ApproachWn;
            float c = 2f * mRed * ApproachWn;
            Vector3 sepVec = (Vector3)(rA - rB); // A's point relative to B's
            Vector3 dv = (Vector3)(a.PointVelocityWorld - b.PointVelocityWorld);
            Vector3 force = -k * sepVec - c * dv;
            float fmax = mRed * ApproachAmax;
            Vector3 impulseA = CapImpulse(force * (float)TickClock.FixedStep, fmax * (float)TickClock.FixedStep);
            if (impulseA != Vector3.Zero)
            {
                ba.ApplyLinearImpulse(impulseA);
            }
            if (impulseA != Vector3.Zero)
            {
                bb.ApplyLinearImpulse(-impulseA);
            }

            // Alignment: rotate each grid so its connect axis becomes
            // anti-parallel to the other's (the approach aligns opposing
            // axes before the final lock).
            AlignAxis(ba, axisA, -axisB);
            AlignAxis(bb, axisB, -axisA);
        }

        private static Vector3 CapImpulse(Vector3 v, float cap)
        {
            float len = v.Length();
            return len > cap ? v / len * cap : v;
        }

        private static void AlignAxis(SimRigidBody body, Vector3D axis, Vector3D desired)
        {
            Vector3 cur = (Vector3)Vector3D.Normalize(axis);
            Vector3 des = (Vector3)Vector3D.Normalize(desired);
            Vector3 cross = Vector3.Cross(cur, des);
            float crossLen = cross.Length();
            if (crossLen < 1e-9f)
            {
                return;
            }
            float angle = (float)Math.Asin(ClampF(crossLen, -1f, 1f));
            Vector3 axisRot = cross / crossLen;
            Vector3 desAngVel = axisRot * Math.Min(ApproachAngularGain * angle, ApproachAngularCap);
            Vector3 delta = (desAngVel - (Vector3)body.AngularVelocity) * ApproachAngularStep;
            if (delta.Length() > ApproachAngularCap)
            {
                delta = Vector3.Normalize(delta) * ApproachAngularCap;
            }
            // Scalar inertia estimate from the inverse-inertia diagonal
            // mean; drives the axis toward anti-parallel (the lock's
            // angular tolerance absorbs the residual).
            Vector3 invDiag = body.InverseInertiaDiagonal;
            float invMean = (invDiag.X + invDiag.Y + invDiag.Z) / 3f;
            float inertia = invMean > 1e-6f ? 1f / invMean : 1f;
            body.ApplyAngularImpulse(delta * inertia);
        }

        /// <summary>
        /// Tick phase 8: once an approaching pair has converged (separation
        /// within the profile's approach gap and axes anti-parallel within
        /// the approach angle, at low relative velocity), the master creates
        /// the final fixed constraint and both become Connected.
        /// </summary>
        public static void AdvanceApproach(IReadOnlyList<SimShipConnector> connectors, SePhysicsProfile profile)
        {
            if (connectors == null || profile == null)
            {
                return;
            }
            for (int i = 0; i < connectors.Count; i++)
            {
                SimShipConnector c = connectors[i];
                if (c.Closed || !c.ApproachingInternal)
                {
                    continue;
                }
                if (!ReferenceEquals(c.ApproachMasterInternal, c))
                {
                    continue;
                }
                SimShipConnector slave = c.ApproachPartnerInternal;
                if (slave == null || slave.Closed)
                {
                    continue;
                }
                if (Converged(c, slave, profile))
                {
                    c.LockTo(slave);
                }
            }
        }

        private static bool Converged(SimShipConnector a, SimShipConnector b, SePhysicsProfile profile)
        {
            float separation = (float)(a.WorldPosition - b.WorldPosition).Length();
            if (separation > profile.ConnectorApproachGap)
            {
                return false;
            }
            double angle = AngleBetween(a.ConnectAxisWorld, b.ConnectAxisWorld);
            double angularError = Math.Abs(Math.PI - angle);
            if (angularError > profile.ConnectorApproachAngle)
            {
                return false;
            }
            double relativeVelocity = (a.PointVelocityWorld - b.PointVelocityWorld).Length();
            return relativeVelocity <= profile.ConnectorLinearVelocityMax;
        }

        private static float ClampF(float v, float lo, float hi)
        {
            return v < lo ? lo : v > hi ? hi : v;
        }
    }
}
