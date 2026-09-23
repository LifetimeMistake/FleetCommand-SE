using System;
using System.Collections.Generic;
using FleetCommand.Common;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The connector docking capture logic: the deterministic, configurable capture gates
    /// that decide when two unconnected connectors become <c>Connectable</c>. The game's live
    /// docking search is a complex physics proximity scan; the framework models capture as four
    /// discrete gates (separation, opposing-axis angular error, relative linear velocity at the
    /// connector points, relative angular velocity), with thresholds from the world's
    /// <see cref="SePhysicsProfile"/>, so docking tests can distinguish approach convergence,
    /// capture readiness, and an actual connection deterministically. Driven each tick by
    /// <see cref="SimWorld"/> in phase 5; the <see cref="SimShipConnector"/> stores the
    /// mutually-captured result (capture always pairs peer connectors so an eligible pair is both
    /// Connectable).</summary>
    internal static class ConnectorSystem
    {
        /// <summary>Evaluates the capture gates over all unconnected connectors in deterministic
        /// order, marking eligible unconnected pairs as mutually captured. Connectors on the same
        /// grid are never docking partners (the framework models cross-grid docking).</summary>
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

        /// <summary>The four capture gates. A pair passes when all of: separation ≤ min of the two
        /// connectors' effective reach and the profile reach; opposing-axis angular error ≤
        /// <c>profile.ConnectorAngularErrorMax</c>; relative linear velocity at the connector
        /// points ≤ <c>profile.ConnectorLinearVelocityMax</c>; relative angular velocity ≤
        /// <c>profile.ConnectorAngularVelocityMax</c>.</summary>
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
            double angle = MathHelpers.AngleBetween(a.ConnectAxisWorld, b.ConnectAxisWorld);
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

        // — the two-stage connect (approach then lock) —

        // Approach convergence (deterministic, fixed 1/60 s step): a
        // physical, under-damped spring between the two connector
        // connection frames — force at each frame (via the reduced mass,
        // natural frequency wn with damping c), applied at the frames
        // themselves (off the center of mass). Applying the force at r
        // yields, per body, BOTH a linear impulse and the angular
        // impulse τ = r × F, so the pull translates the bodies together
        // AND, being off-COM, rotates them into alignment as a
        // consequence of the coupled dynamics — exactly the game's
        // CreateConstraint magnet tied at the connector frames. No
        // separate alignment controller: rotation emerges from r × F.
        private const float ApproachWn = 2f;      // magnet spring natural frequency (rad/s)
        private const float ApproachAmax = 6f;    // max pull acceleration (m/s^2)
        private const float ApproachDampingRatio = 1f;  // near-critically damped
        // The fixed frame constraint's rotational tie (the torsional
        // spring that restores the two connection frames to coincident
        // orientation), as a torsion spring on the relative
        // frame-orientation error with the reduced rotational inertia.
        private const float ApproachRotWn = 1.2f;     // torsion natural frequency (rad/s)
        private const float ApproachRotDamping = 1f;
        private const float ApproachRotAmax = 1.5f;   // max torsion acceleration (rad/s^2)

        /// <summary>The magnet spring natural frequency (rad/s).</summary>
        internal static float MagnetNaturalFrequency => ApproachWn;

        /// <summary>The magnet's maximum pull acceleration (m/s^2).</summary>
        internal static float MagnetMaxAcceleration => ApproachAmax;

        /// <summary>The magnet spring damping ratio.</summary>
        internal static float MagnetDampingRatio => ApproachDampingRatio;

        /// <summary>The frame-tie torsion natural frequency (rad/s).</summary>
        internal static float FrameTieNaturalFrequency => ApproachRotWn;

        /// <summary>The frame-tie torsion damping ratio.</summary>
        internal static float FrameTieDampingRatio => ApproachRotDamping;

        /// <summary>The frame-tie maximum torsion acceleration (rad/s^2).</summary>
        internal static float FrameTieMaxAcceleration => ApproachRotAmax;

        /// <summary>Starts the approach for a connectable pair (the game's
        /// <c>TryAttach</c>→<c>CreateConstraint</c>, MyShipConnector.cs:1478-1520, :1882): the
        /// master (higher EntityId, the game's <c>GetMaster</c>) drives the convergence. If the
        /// pair is already effectively touching and aligned, it locks immediately
        /// instead.</summary>
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

        /// <summary>Cancels an in-progress approach (release): both connectors return to a clean,
        /// unconnected, free state (no residual constraint).</summary>
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

        /// <summary>Tick phase 6: apply the approach convergence — a pull on each body at its
        /// connector connection frame, driving the connector frames to coincidence — matching the
        /// game's <c>CreateConstraint</c> magnet (a fixed constraint at the two connector frames,
        /// MyShipConnector.cs:1882). The off-COM force couples translation and rotation: the torque
        /// r × F rotates the connect axes into alignment as a consequence of the dynamics, not by
        /// command. Only the master drives; deterministic.</summary>
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

            Vector3D rA = a.WorldPosition;   // A's connection frame
            Vector3D rB = b.WorldPosition;   // B's connection frame

            // The magnet: an under-damped spring between the two
            // connection frames, on the reduced mass — force at A's frame
            // pulling it to B's, and its equal-and-opposite reaction at
            // B's frame. Applied at the frames (off-COM), each force
            // delivers both motion of the center of mass and the torque
            // r × F, so coupling (translate + rotate into alignment)
            // falls out of r × F naturally — no separate alignment
            // controller.
            float mRed = (ba.Mass * bb.Mass) / (ba.Mass + bb.Mass);
            float k = mRed * ApproachWn * ApproachWn;
            float c = 2f * ApproachDampingRatio * mRed * ApproachWn;
            Vector3 sepVec = (Vector3)(rA - rB); // A's frame relative to B's
            float sepLen = sepVec.Length();
            Vector3 dv = (Vector3)(a.PointVelocityWorld - b.PointVelocityWorld);
            if (sepLen < 1e-6f)
            {
                return;
            }
            Vector3 sepDir = sepVec / sepLen;
            // A central spring along the line between the two frames:
            // the damping only acts along the separation axis (the
            // relative linear velocity projected onto it), so the two
            // equal-and-opposite point forces are collinear and the
            // pair's internal forces conserve angular momentum — no
            // net spin is injected (the game's connector pull is along
            // the connector axis).
            float dvAlong = Vector3.Dot(dv, sepDir);
            Vector3 force = -(k * sepLen + c * dvAlong) * sepDir;
            float fmax = mRed * ApproachAmax;
            Vector3 impulseA = CapImpulse(force * (float)TickClock.FixedStep, fmax * (float)TickClock.FixedStep);
            if (impulseA != Vector3.Zero)
            {
                ba.ApplyPointImpulse(impulseA, rA);   // linear + r_A × F (off-COM torque)
            }
            if (impulseA != Vector3.Zero)
            {
                bb.ApplyPointImpulse(-impulseA, rB);  // linear + r_B × (−F) (off-COM torque)
            }

            // The fixed frame constraint's rotational tie: a torsion
            // spring that restores the two connection frames to
            // coincident orientation (the connect axes anti-parallel).
            // This is the frame constraint's orientational DOF — the
            // counterpart to the positional spring above, with each
            // body's actual rotational inertia.
            FrameRotationalTie(a, b, ba, bb, (float)TickClock.FixedStep);
        }

        /// <summary>Restores the two connection frames to coincident orientation: a torsion spring
        /// on the RELATIVE frame-orientation error (target: the connect axes anti-parallel),
        /// applied as an equal-and-opposite angular impulse to the two bodies scaled by the reduced
        /// rotational inertia about the error axis. Because the correction acts only on the
        /// relative DOF and adds no net angular impulse, it is intrinsically stable — the physical
        /// counterpart of the positional magnet (the fixed constraint ties both position and
        /// orientation of the frames).</summary>
        private static void FrameRotationalTie(
            SimShipConnector a, SimShipConnector b, SimRigidBody ba, SimRigidBody bb, float dt)
        {
            Vector3 axisA = (Vector3)Vector3D.Normalize(a.ConnectAxisWorld);
            Vector3 axisB = (Vector3)Vector3D.Normalize(b.ConnectAxisWorld);
            // Error axis: rotate A's connect axis onto anti-parallel of B's.
            Vector3 cross = Vector3.Cross(axisA, -axisB);
            float s = cross.Length();
            if (s < 1e-9f)
            {
                return;
            }
            float angle = (float)Math.Asin(s > 1f ? 1f : s);
            Vector3 n = cross / s;

            // Relative angular rate and reduced rotational inertia about
            // the error axis.
            Vector3 wRel = (Vector3)(ba.AngularVelocity - bb.AngularVelocity);
            float rate = Vector3.Dot(wRel, n);
            float iA = ScalarInertia(ba);
            float iB = ScalarInertia(bb);
            float iRed = (iA * iB) / (iA + iB);

            float k = iRed * ApproachRotWn * ApproachRotWn;
            float cd = 2f * ApproachRotDamping * iRed * ApproachRotWn;
            float torque = k * angle + cd * rate;
            float torqueMax = iRed * ApproachRotAmax;
            Vector3 impulse = CapImpulse(n * (torque * dt), torqueMax * dt);
            if (impulse == Vector3.Zero)
            {
                return;
            }
            ba.ApplyAngularImpulse(impulse);   // + on A
            bb.ApplyAngularImpulse(-impulse);  // reaction on B (no net torque)
        }

        private static float ScalarInertia(SimRigidBody body)
        {
            Vector3 inv = body.InverseInertiaDiagonal;
            float invMean = (inv.X + inv.Y + inv.Z) / 3f;
            return invMean > 1e-6f ? 1f / invMean : 1f;
        }

        private static Vector3 CapImpulse(Vector3 v, float cap)
        {
            float len = v.Length();
            return len > cap ? v / len * cap : v;
        }

        /// <summary>Tick phase 8: once an approaching pair has converged (separation within the
        /// profile's approach gap and axes anti-parallel within the approach angle, at low relative
        /// velocity), the master creates the final fixed constraint and both become
        /// Connected.</summary>
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
            double angle = MathHelpers.AngleBetween(a.ConnectAxisWorld, b.ConnectAxisWorld);
            double angularError = Math.Abs(Math.PI - angle);
            if (angularError > profile.ConnectorApproachAngle)
            {
                return false;
            }
            double relativeVelocity = (a.PointVelocityWorld - b.PointVelocityWorld).Length();
            return relativeVelocity <= profile.ConnectorLinearVelocityMax;
        }
    }
}
