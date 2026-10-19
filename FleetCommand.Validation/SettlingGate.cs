using System;
using VRageMath;

namespace FleetCommand.Validation
{
    public enum SettleDecision { Brake, BeginCoast, Coast, Ready }

    public sealed class SettlingGate
    {
        public const double SpeedLimit = 0.01, AngularLimit = 0.01, DriftLimit = 0.01, RotationLimit = 0.01;
        public const int BrakeFrames = 15, CoastFrames = 60;
        Vector3D _previousPosition, _coastOrigin;
        QuaternionD _previousAttitude, _coastAttitude;
        bool _hasPrevious, _coasting;
        public int StableTicks { get; private set; }
        public int PassiveTicks { get; private set; }
        public double ObservedSpeed { get; private set; }
        public double ObservedAngularSpeed { get; private set; }
        public double PassiveDrift { get; private set; }
        public double PassiveRotation { get; private set; }
        public void Reset()
        {
            StableTicks = 0; PassiveTicks = 0; _hasPrevious = false; _coasting = false;
            ObservedSpeed = 0; ObservedAngularSpeed = 0; PassiveDrift = 0; PassiveRotation = 0;
        }
        public SettleDecision Observe(Vector3D position, QuaternionD attitude, Vector3D referenceVelocity,
            Vector3D cockpitVelocity, Vector3D angularVelocity, double dt)
        {
            ObservedSpeed = _hasPrevious ? Vector3D.Distance(position, _previousPosition) / dt : 0;
            ObservedAngularSpeed = _hasPrevious ? Angle(attitude, _previousAttitude) / dt : 0;
            _previousPosition = position; _previousAttitude = attitude; _hasPrevious = true;
            bool quiet = referenceVelocity.Length() < SpeedLimit && cockpitVelocity.Length() < SpeedLimit
                && angularVelocity.Length() < AngularLimit && ObservedSpeed < SpeedLimit && ObservedAngularSpeed < AngularLimit;
            if (_coasting)
            {
                PassiveDrift = Vector3D.Distance(position, _coastOrigin);
                PassiveRotation = Angle(attitude, _coastAttitude);
                if (!quiet || PassiveDrift > DriftLimit || PassiveRotation > RotationLimit)
                { _coasting = false; StableTicks = 0; PassiveTicks = 0; return SettleDecision.Brake; }
                PassiveTicks++;
                return PassiveTicks >= CoastFrames ? SettleDecision.Ready : SettleDecision.Coast;
            }
            StableTicks = quiet ? StableTicks + 1 : 0;
            if (StableTicks < BrakeFrames) return SettleDecision.Brake;
            _coasting = true; PassiveTicks = 0; PassiveDrift = 0; PassiveRotation = 0;
            _coastOrigin = position; _coastAttitude = attitude;
            return SettleDecision.BeginCoast;
        }

        static double Angle(QuaternionD a, QuaternionD b)
        {
            double dot = Math.Abs(a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W);
            return 2 * Math.Acos(Math.Min(1, dot));
        }
    }
}
