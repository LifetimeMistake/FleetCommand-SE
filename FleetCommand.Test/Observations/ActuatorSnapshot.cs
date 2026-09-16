using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.SpaceEngineers;
using VRageMath;

namespace FleetCommand.Test.Observations
{
    /// <summary>
    /// The per-tick per-grid actuator and state snapshot the world keeps in
    /// its short failure ring buffer ( "recent actuator history"):
    /// the delivered net thrust, the requested gyro target rate, the
    /// delivered torque, and gyro saturation, at a completed tick. This is
    /// the recent-history data the fixture surfaces when a test fails. It is
    /// a plain value; rendering is <see cref="ToLine"/>.
    /// </summary>
    public struct ActuatorSnapshot
    {
        /// <summary>The tick this snapshot was captured at.</summary>
        public long Tick;

        /// <summary>Simulation time (s) of this snapshot.</summary>
        public double Time;

        /// <summary>The grid id this snapshot describes.</summary>
        public long GridId;

        /// <summary>Delivered net thrust (body frame, N) from the last aggregation.</summary>
        public Vector3 NetForce;

        /// <summary>Requested override target rate (grid frame, rad/s).</summary>
        public Vector3 RequestedGyroRate;

        /// <summary>Delivered torque (grid frame, N·m) applied in the last interval.</summary>
        public Vector3 DeliveredTorque;

        /// <summary>True when the last applied torque was clamped by the gyro-force sphere.</summary>
        public bool Saturated;

        /// <summary>Captures the grid's current actuator state ().</summary>
        public static ActuatorSnapshot Capture(SimCubeGrid grid, long tick, double time)
        {
            return new ActuatorSnapshot
            {
                Tick = tick,
                Time = time,
                GridId = grid.EntityId,
                NetForce = grid.ForceAggregator.LastNetForce,
                RequestedGyroRate = grid.GyroRateSystem.OverrideTargetVelocity,
                DeliveredTorque = grid.GyroRateSystem.LastTorque,
                Saturated = grid.GyroRateSystem.LastSaturated,
            };
        }

        /// <summary>A one-line diagnostic summary.</summary>
        public string ToLine()
        {
            return "tick " + Tick + " grid " + GridId
                + " thrust=" + VectorText(NetForce)
                + " gyroRate=" + VectorText(RequestedGyroRate)
                + " torque=" + VectorText(DeliveredTorque)
                + " sat=" + (Saturated ? "1" : "0");
        }

        private static string VectorText(Vector3 v)
        {
            return "(" + v.X.ToString("R") + "," + v.Y.ToString("R") + "," + v.Z.ToString("R") + ")";
        }
    }
}
