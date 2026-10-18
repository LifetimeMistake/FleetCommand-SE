using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.Sbc
{
    /// <summary>
    /// Base-six direction handling for SBC blueprints. Two name spaces meet here: the
    /// game's <c>BlockOrientation</c> direction names (Forward/Backward/Left/Right/
    /// Up/Down) and the local mount-face names used by block definitions
    /// (Front/Back/Top/Bottom/Left/Right). Both reduce to the same six small integer
    /// cell directions; this class owns that table plus the per-block frame math
    /// (grid cells a block occupies, and which world faces it can mount through) so the
    /// reader, the validator and the writer all agree.
    /// </summary>
    public static class CubeFacing
    {
        // BlockOrientation (base-six) direction names.
        public const string OrientForward = "Forward";
        public const string OrientBackward = "Backward";
        public const string OrientLeft = "Left";
        public const string OrientRight = "Right";
        public const string OrientUp = "Up";
        public const string OrientDown = "Down";

        // Local mount-face names.
        public const string FaceFront = "Front";
        public const string FaceBack = "Back";
        public const string FaceTop = "Top";
        public const string FaceBottom = "Bottom";
        public const string FaceLeft = "Left";
        public const string FaceRight = "Right";

        /// <summary>The six internal face names (a stable key set for
        /// <see cref="BlockGeometry"/> and the mount logic).</summary>
        public static readonly IReadOnlyList<string> AllFaces = new[]
        {
            FaceFront, FaceBack, FaceLeft, FaceRight, FaceTop, FaceBottom,
        };

        /// <summary>True for the block-orientation direction names (Forward etc.).</summary>
        public static bool IsOrientName(string name)
        {
            return OrientToFace.ContainsKey(name);
        }

        // internal face name -> the cell's integer direction.
        private static readonly Dictionary<string, Vector3I> Dir = new Dictionary<string, Vector3I>
        {
            { FaceFront, new Vector3I(0, 0, -1) },
            { FaceBack, new Vector3I(0, 0, 1) },
            { FaceLeft, new Vector3I(-1, 0, 0) },
            { FaceRight, new Vector3I(1, 0, 0) },
            { FaceTop, new Vector3I(0, 1, 0) },
            { FaceBottom, new Vector3I(0, -1, 0) },
        };

        // BlockOrientation name -> internal face name.
        private static readonly Dictionary<string, string> OrientToFace = new Dictionary<string, string>
        {
            { OrientForward, FaceFront },
            { OrientBackward, FaceBack },
            { OrientLeft, FaceLeft },
            { OrientRight, FaceRight },
            { OrientUp, FaceTop },
            { OrientDown, FaceBottom },
        };

        /// <summary>Maps a block-orientation direction name (e.g. "Forward") to its
        /// internal face name (e.g. "Front").</summary>
        public static string OrientToFaceName(string orientName)
        {
            if (!OrientToFace.TryGetValue(orientName, out string face))
            {
                throw new ArgumentException(
                    $"Unknown block-orientation direction '{orientName}'.", nameof(orientName));
            }
            return face;
        }

        internal static Vector3I DirOf(string face) => Dir[face];

        private static Vector3I Cross(Vector3I a, Vector3I b)
        {
            return new Vector3I(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        }

        private static Vector3I Neg(Vector3I v) => new Vector3I(-v.X, -v.Y, -v.Z);

        /// <summary>The orthogonal world frame of a block given its orientation as two
        /// internal face names (forward, up): <c>Right = fw × up</c>, <c>Back = -fw</c>
        /// — the same handedness the game uses (a default -Z forward gives +X right).</summary>
        public static BlockFrame MakeFrame(string forwardFace, string upFace)
        {
            Vector3I fw = Dir[forwardFace];
            Vector3I up = Dir[upFace];
            Vector3I right = Cross(fw, up);
            return new BlockFrame(right, up, Neg(fw));
        }

        /// <summary>Nearest internal face name for a world direction.</summary>
        internal static string FaceOfDir(Vector3I v)
        {
            string best = null;
            long bestDist = long.MaxValue;
            foreach (string face in AllFaces)
            {
                Vector3I d = Dir[face];
                long dist = (long)(d.X - v.X) * (d.X - v.X)
                          + (long)(d.Y - v.Y) * (d.Y - v.Y)
                          + (long)(d.Z - v.Z) * (d.Z - v.Z);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    best = face;
                }
            }
            return best;
        }
    }

    /// <summary>
    /// A block's resolved world frame (right, up, back) plus the derived geometry:
    /// the set of grid cells a block occupies and the set of world faces it can mount
    /// through (its local mount faces mapped through the frame). Matches the game's
    /// placement and the equivalence vessel.
    /// </summary>
    public struct BlockFrame
    {
        private readonly Vector3I _right;
        private readonly Vector3I _up;
        private readonly Vector3I _back;

        /// <summary>The block's world Back direction, i.e. the negative of its
        /// placement Forward.</summary>
        public Vector3I Back => _back;

        /// <summary>The block's world Forward direction (opposite of <see cref="Back"/>).</summary>
        public Vector3I Forward => new Vector3I(-_back.X, -_back.Y, -_back.Z);

        /// <summary>The block's world Up direction.</summary>
        public Vector3I Up => _up;

        /// <summary>The block's world Right direction.</summary>
        public Vector3I Right => _right;

        internal BlockFrame(Vector3I right, Vector3I up, Vector3I back)
        {
            _right = right;
            _up = up;
            _back = back;
        }

        /// <summary>The world direction of a local face (Front/Back/Top/Bottom/Left/
        /// Right) given this frame.</summary>
        public Vector3I FaceWorldDir(string face)
        {
            switch (face)
            {
                case CubeFacing.FaceFront: return new Vector3I(-_back.X, -_back.Y, -_back.Z);
                case CubeFacing.FaceBack: return _back;
                case CubeFacing.FaceTop: return _up;
                case CubeFacing.FaceBottom: return new Vector3I(-_up.X, -_up.Y, -_up.Z);
                case CubeFacing.FaceRight: return _right;
                case CubeFacing.FaceLeft: return new Vector3I(-_right.X, -_right.Y, -_right.Z);
                default: throw new ArgumentException($"Unknown face '{face}'.", nameof(face));
            }
        }
    }
}
