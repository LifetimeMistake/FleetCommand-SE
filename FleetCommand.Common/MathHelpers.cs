using System;
using VRageMath;

namespace FleetCommand.Common
{
    /// <summary>
    /// Small, reused vector math helpers shared by the simulation systems.
    /// Pure functions. This file is part of <c>FleetCommand.Common</c>, so
    /// it is compiled into BOTH the packed game script (C# 6, LangVersion
    /// pinned) and the test assembly.
    /// </summary>
    public static class MathHelpers
    {
        /// <summary>
        /// The game's ratio-with-invalid-guard: n/d, or 1 when the
        /// denominator is zero or the quotient is NaN (the game's
        /// !IsValid() -> 1).
        /// </summary>
        public static float SafeRatio(float numerator, float denominator)
        {
            if (denominator == 0f || float.IsNaN(numerator / denominator))
            {
                return 1f; // the game's !IsValid() -> 1
            }
            return numerator / denominator;
        }

        /// <summary>Component-wise <see cref="SafeRatio(float,float)"/>.</summary>
        public static Vector3 SafeRatio(Vector3 a, Vector3 b)
        {
            return new Vector3(
                SafeRatio(a.X, b.X),
                SafeRatio(a.Y, b.Y),
                SafeRatio(a.Z, b.Z));
        }

        /// <summary>
        /// Normalizes a per-axis force back to a grid-body group demand,
        /// dividing each axis by the max thrust that axis can deliver in
        /// the direction the force points (positive or negative). A zero
        /// force, or a direction with no capacity, maps to 0 on that
        /// axis.
        /// </summary>
        public static Vector3 Normalize(Vector3 force, Vector3 maxPos, Vector3 maxNeg)
        {
            return new Vector3(
                NormalizeAxis(force.X, force.X >= 0f ? maxPos.X : maxNeg.X),
                NormalizeAxis(force.Y, force.Y >= 0f ? maxPos.Y : maxNeg.Y),
                NormalizeAxis(force.Z, force.Z >= 0f ? maxPos.Z : maxNeg.Z));
        }

        private static float NormalizeAxis(float force, float max)
        {
            if (force == 0f)
            {
                return 0f;
            }
            if (max == 0f)
            {
                return 0f;
            }
            return force / max;
        }

        /// <summary>
        /// The angle (rad, in [0, pi]) between two world directions,
        /// with the dot product clamped to [-1, 1] before the inverse
        /// cosine (numerically robust against near-parallel inputs).
        /// </summary>
        public static double AngleBetween(Vector3D a, Vector3D b)
        {
            double dot = Vector3D.Dot(Vector3D.Normalize(a), Vector3D.Normalize(b));
            dot = dot < -1.0 ? -1.0 : dot > 1.0 ? 1.0 : dot;
            return Math.Acos(dot);
        }
    }
}
