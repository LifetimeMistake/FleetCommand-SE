using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
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
    }
}
