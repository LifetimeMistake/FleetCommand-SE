using VRageMath;

namespace FleetCommand.Validation
{
    /// <summary>Fixed controller frame captured at run start; never follows the craft.</summary>
    public sealed class InitialFrame
    {
        readonly MatrixD _worldToInitial;
        readonly Vector3D _origin;

        public InitialFrame(MatrixD orientation, Vector3D centerOfMass)
        {
            _worldToInitial = MatrixD.Invert(orientation.GetOrientation());
            _origin = centerOfMass;
        }

        public Vector3D Position(Vector3D worldPosition)
        {
            return Vector(worldPosition - _origin);
        }

        public Vector3D Vector(Vector3D worldVector)
        {
            return Vector3D.TransformNormal(worldVector, _worldToInitial);
        }

        public QuaternionD Attitude(MatrixD orientation)
        {
            var relative = orientation.GetOrientation() * _worldToInitial;
            var q = QuaternionD.CreateFromRotationMatrix(relative);
            q.Normalize();
            // q and -q encode the same rotation: use one canonical representative.
            if (q.W < 0) q = new QuaternionD(-q.X, -q.Y, -q.Z, -q.W);
            return q;
        }
    }
}
