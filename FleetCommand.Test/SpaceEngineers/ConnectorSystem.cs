using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>The connector docking lifecycle: a faithful port of the game's algorithm from
    /// <c>MyShipConnector.UpdateBeforeSimulation10</c> (auto-detect + TryAttach) plus the
    /// approach-keep — the real game uses Havok's malleable hinge for the
    /// approach and a fixed constraint for the weld, both authored here and driven by the world's
    /// Havok solver; there is no hand-written spring. Called each tick by <see cref="SimWorld"/>
    /// in phase 5 (after power recompute, so <see cref="SimFunctionalBlock.IsWorking"/> is
    /// fresh); <see cref="SimShipConnector"/> stores the paired/connected result.</summary>
    internal static class ConnectorSystem
    {
        /// <summary>The auto-attach separation gate (m^2): the two connection positions must be
        /// within this before TryAttach creates the approach constraint — the game's literal
        /// <c>LengthSquared() &lt; 0.3499999940395355</c> (MyShipConnector.cs:1486), so the
        /// faces must be within ≈0.59 m. Kept configurable via the profile.</summary>
        internal static float AttachSeparationSquared(SePhysicsProfile profile)
        {
            return profile.ConnectorAttachSeparationSquared;
        }

        /// <summary>The approach-keep gate (m^2): while approaching (not yet welded) the pair
        /// detaches if the connection positions drift beyond this — the game's literal
        /// <c>LengthSquared() &gt; 0.5</c> (MyShipConnector.cs:1574), so the pair must stay
        /// within ≈0.7 m. Kept configurable via the profile.</summary>
        internal static float KeepSeparationSquared(SePhysicsProfile profile)
        {
            return profile.ConnectorKeepSeparationSquared;
        }

        /// <summary>Runs the connector lifecycle every ten physics ticks over the deterministic registry —
        /// the game's <c>UpdateBeforeSimulation10</c> (:1537-1580): detach a now-unworking or
        /// drifted pair, maintain the approach's malleable strength, and
        /// auto-attach unpaired working connectors whose faces are within range.</summary>
        public static void Tick(IReadOnlyList<SimShipConnector> connectors, SePhysicsProfile profile)
        {
            if (connectors == null)
            {
                return;
            }
            for (int i = 0; i < connectors.Count; i++)
            {
                SimShipConnector c = connectors[i];
                if (c.Closed)
                {
                    continue;
                }
                if ((c.Grid.World.Clock.Tick + 1) % 10 != 0)
                {
                    continue;
                }
                if (!c.IsWorking)
                {
                    // Game :1548-1558: a not-working connector detaches whenever it is in a
                    // constraint and not yet welded, or welded once it is no longer functional or
                    // enabled (a purely unpowered but otherwise-sound weld is kept).
                    if (c.InConstraint && (!c.ConnectedInternal || !c.IsFunctional || !c.Enabled))
                    {
                        c.DetachLink();
                    }
                    continue;
                }
                bool shouldTryAttach = c.AdvanceWorkingUpdate10();
                if (c.InConstraint)
                {
                    if (!c.ConnectedInternal)
                    {
                        // Approach-keep (:1559-1578): the master live-updates the malleable
                        // strength to the pair's effective value; if the faces drift beyond the
                        // keep range the pair came apart and is detached. Contact never welds;
                        // only explicit Connect/TryConnect or replicated state calls Weld.
                        if (c.IsMaster)
                        {
                            c.UpdateApproachStrength();
                        }
                        SimShipConnector other = c.PartnerInternal;
                        float sepSq = (float)(c.WorldPosition - other.WorldPosition).LengthSquared();
                        if (sepSq > KeepSeparationSquared(profile))
                        {
                            c.DetachLink();
                        }
                    }
                    continue;
                }
                // Not in a constraint and working: try to attach, exactly as the game's
                // UpdateBeforeSimulation10 drove TryAttach for working, Enabled,
                // connector-mode connectors (:1543-1546).
                if (c.Enabled && shouldTryAttach)
                {
                    TryAttach(c, connectors, profile);
                }
            }
        }

        /// <summary>The game's <c>TryAttach</c> (MyShipConnector.cs:1478-1517): find another
        /// connector within detection reach; when it passes the attach gates, the master
        /// (<c>GetMaster</c>) creates the malleable approach constraint.</summary>
        private static void TryAttach(SimShipConnector c, IReadOnlyList<SimShipConnector> connectors, SePhysicsProfile profile)
        {
            SimShipConnector other = FindOtherConnector(c, connectors, profile);
            if (other == null)
            {
                return;
            }
            if (AttachPasses(c, other, profile))
            {
                // The master (GetMaster) creates the approach constraint with the other
                // connector as the slave (game TryAttach :1494-1505: if master==this,
                // CreateConstraint(the other); else the other creates it with this).
                SimShipConnector master = GetMaster(c, other);
                SimShipConnector slave = ReferenceEquals(master, c) ? other : c;
                master.AttachAsMaster(slave);
            }
        }

        /// <summary>Finds this connector's pairing candidate: the nearest other connector within
        /// its detector sphere (the game's <c>FindOtherConnector</c>, MyShipConnector.cs:1826 —
        /// searches the detector sphere over the detected grids, skipping the same grid and
        /// already-paired or non-working connectors). Deterministic (nearest; ties broken by
        /// registry order).</summary>
        private static SimShipConnector FindOtherConnector(
            SimShipConnector c, IReadOnlyList<SimShipConnector> connectors, SePhysicsProfile profile)
        {
            double radius = c.DetectorRadius;
            double bestSq = double.MaxValue;
            SimShipConnector best = null;
            for (int i = 0; i < connectors.Count; i++)
            {
                SimShipConnector candidate = connectors[i];
                if (candidate.Closed || candidate == c)
                {
                    continue;
                }
                if (candidate.InConstraint || !candidate.IsWorking)
                {
                    continue;
                }
                if (ReferenceEquals(candidate.Grid, c.Grid))
                {
                    continue; // never pair within one grid
                }
                double d = (candidate.WorldPosition - c.WorldPosition).LengthSquared();
                if (d <= radius * radius && d < bestSq)
                {
                    bestSq = d;
                    best = candidate;
                }
            }
            return best;
        }

        /// <summary>The attach gates (MyShipConnector.cs:1486-1497): the partner is a
        /// (connector-mode) working connector whose connection face is within the attach
        /// separation, whose port has the same size (<c>IsSmallConnector == IsSmallConnector</c>),
        /// and whose connect directions are opposed within 45° (<c>Angle(a,b) &gt; 3π/4</c>).
        /// Both connectors must also have live physics.</summary>
        private static bool AttachPasses(SimShipConnector a, SimShipConnector b, SePhysicsProfile profile)
        {
            if (a.Grid.Body == null || b.Grid.Body == null || a.Grid.Body.IsDisposed || b.Grid.Body.IsDisposed)
            {
                return false;
            }
            // Port size comes from the detector dummy name, independently of grid size (:1284).
            if (a.IsSmallConnector != b.IsSmallConnector)
            {
                return false;
            }
            // Separation gate (:1486).
            Vector3D sep = b.WorldPosition - a.WorldPosition;
            if (sep.LengthSquared() >= profile.ConnectorAttachSeparationSquared)
            {
                return false;
            }
            // Opposing-angle gate (:1488-1492): the connect directions must be opposed within 45°.
            Vector3D aAxis = Vector3D.Normalize(a.AttachDirectionWorld);
            Vector3D bAxis = Vector3D.Normalize(b.AttachDirectionWorld);
            double angle = Vector3D.Angle(aAxis, bAxis);
            double minOpposed = Math.PI * 3.0 / 4.0;
            if (angle < minOpposed)
            {
                return false;
            }
            return true;
        }

        /// <summary>The game's <c>GetMaster</c> (MyShipConnector.cs:2435-2466): prefers the
        /// non-static grid, then the smaller grid, then the higher EntityId. For the sim's
        /// equal-dynamic, equal-size pairs this selects the higher EntityId.</summary>
        private static SimShipConnector GetMaster(SimShipConnector first, SimShipConnector second)
        {
            SimCubeGrid g1 = first.Grid;
            SimCubeGrid g2 = second.Grid;
            if (g1.IsStatic != g2.IsStatic)
            {
                return g1.IsStatic ? second : first;
            }
            if (g1.GridSize != g2.GridSize)
            {
                if (g1.GridSize > 1f)
                {
                    return second;
                }
                if (g2.GridSize > 1f)
                {
                    return first;
                }
            }
            return first.EntityId >= second.EntityId ? first : second;
        }
    }
}
