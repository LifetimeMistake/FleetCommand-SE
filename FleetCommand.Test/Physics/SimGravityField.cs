using System;
using VRageMath;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// Natural gravity, owned by <see cref="Framework.SimWorld"/> (:
    /// "Natural gravity is owned by SimGravityField, not a planet
    /// implementation"). The same vector this field calculates for a world
    /// point is returned by <see cref="Sandbox.ModAPI.Ingame.IMyShipController.GetNaturalGravity"/> ()
    /// and is applied to each rigid body during tick phase 6. Initial
    /// profiles: zero gravity, uniform gravity, and analytic point gravity
    /// (mirroring the game's spherical natural-gravity shape,
    /// MySphericalNaturalGravityComponent).
    /// </summary>
    public abstract class SimGravityField
    {
        /// <summary>
        /// The world-frame gravitational acceleration (m/s²) at
        /// <paramref name="worldPoint"/>.
        /// </summary>
        public abstract Vector3 GetNaturalGravity(Vector3D worldPoint);

        /// <summary>Field with no gravity anywhere.</summary>
        public static readonly SimGravityField Zero = new ZeroGravityField();

        /// <summary>A constant world-frame acceleration everywhere.</summary>
        public static SimGravityField Uniform(Vector3 acceleration)
        {
            return new UniformGravityField(acceleration);
        }

        /// <summary>
        /// Analytic point (spherical) gravity about <paramref name="center"/>:
        /// full strength <c>intensity</c>, with the game's range and falloff
        /// shape (MySphericalNaturalGravityComponent.GetGravityMultiplier).
        /// </summary>
        public static SimGravityField Point(
            Vector3D center,
            double intensity,
            double minRadius,
            double maxRadius,
            double falloff = 2.0)
        {
            return new PointGravityField(center, intensity, minRadius, maxRadius, falloff);
        }

        private sealed class ZeroGravityField : SimGravityField
        {
            public override Vector3 GetNaturalGravity(Vector3D worldPoint)
            {
                return Vector3.Zero;
            }
        }

        private sealed class UniformGravityField : SimGravityField
        {
            private readonly Vector3 _acceleration;

            public UniformGravityField(Vector3 acceleration)
            {
                _acceleration = acceleration;
            }

            public override Vector3 GetNaturalGravity(Vector3D worldPoint)
            {
                return _acceleration;
            }
        }

        private sealed class PointGravityField : SimGravityField
        {
            // Mirrors MySphericalNaturalGravityComponent's constants.
            private const double GravityLimitStrength = 0.05;
            private const double EarthGravity = 9.81;
            private const double MinimumMultiplierClamp = 0.01;

            private readonly Vector3D _center;
            private readonly double _intensity;
            private readonly double _minRadius;
            private readonly double _maxRadius;
            private readonly double _falloff;
            private readonly double _gravityLimit;

            public PointGravityField(
                Vector3D center,
                double intensity,
                double minRadius,
                double maxRadius,
                double falloff)
            {
                _center = center;
                _intensity = intensity;
                _minRadius = minRadius;
                _maxRadius = maxRadius;
                _falloff = falloff;
                _gravityLimit = _maxRadius * Math.Pow(intensity / GravityLimitStrength, 1.0 / falloff);
            }

            public override Vector3 GetNaturalGravity(Vector3D worldPoint)
            {
                double distance = Vector3D.Distance(_center, worldPoint);
                if (distance > _gravityLimit)
                {
                    return Vector3.Zero;
                }

                double multiplier = 1.0;
                if (distance > _maxRadius)
                {
                    multiplier = Math.Pow(distance / _maxRadius, -_falloff);
                }
                else if (distance < _minRadius)
                {
                    multiplier = distance / _minRadius;
                    if (multiplier < MinimumMultiplierClamp)
                    {
                        multiplier = MinimumMultiplierClamp;
                    }
                }
                multiplier *= _intensity;

                Vector3D direction = _center - worldPoint;
                direction.Normalize();
                return (Vector3)(direction * (EarthGravity * multiplier));
            }
        }
    }
}
