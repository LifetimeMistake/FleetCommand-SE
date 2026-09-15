using System;
using System.Collections.Generic;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// The grid's controller rotation control (.6 Part 1): the
    /// rotational analog of <see cref="BaseControlSystem"/>. When a
    /// (under-control, working) ship controller with the
    /// <see cref="SimShipController.ControlGyros"/> gate on writes its
    /// <see cref="SimShipController.RotationIndicator"/> /
    /// <see cref="SimShipController.RollIndicator"/>, this converts them
    /// into a grid-frame angular command exactly as
    /// <c>MyShipController.MoveAndRotate</c> (:1816-1824) does —
    /// <c>RotationIndicator/20</c> (ClampToSphere 1) and <c>RollIndicator · 0.2</c>,
    /// negated, transformed by the block's orientation — and writes it
    /// into <see cref="GyroRateSystem.ControlTorque"/>, the same sink the
    /// game's controller uses (and the autopilot shares).
    /// <para>
    /// Last-under-control-wins matches the game's per-frame assignment
    /// (<c>m_gridGyroSystem.ControlTorque = vector3</c>), not an
    /// accumulation (unlike the translation route's
    /// <c>ControlThrust +=</c>). With no contributing controller the
    /// command is reset to zero so a stale rotation does not persist.
    /// </para>
    /// </summary>
    public sealed class RotationControlSystem
    {
        private readonly SimCubeGrid _grid;
        private readonly List<SimShipController> _controllers = new List<SimShipController>();

        private Vector3 _lastControlTorque;
        private bool _rotationActive;

        /// <summary>The rigid grid (ship) this rotation-control system serves.</summary>
        public SimCubeGrid Grid => _grid;

        /// <summary>The grid-frame command written to <see cref="GyroRateSystem.ControlTorque"/> on the last <see cref="ComputeControlTorque"/>.</summary>
        public Vector3 LastControlTorque => _lastControlTorque;

        /// <summary>True when an under-control controller drove the rotation on the last <see cref="ComputeControlTorque"/>.</summary>
        public bool RotationActive => _rotationActive;

        internal RotationControlSystem(SimCubeGrid grid)
        {
            if (grid == null)
            {
                throw new ArgumentNullException(nameof(grid));
            }
            _grid = grid;
        }

        /// <summary>Registers an under-control-capable ship controller (spawn).</summary>
        internal void RegisterController(SimShipController controller)
        {
            if (controller == null)
            {
                throw new ArgumentNullException(nameof(controller));
            }
            _controllers.Add(controller);
        }

        /// <summary>
        /// Tick phase 5 (before <see cref="GyroRateSystem.ApplyTorque"/>):
        /// reproduce <c>MyShipController.MoveAndRotate</c>'s rotation
        /// block (:1816-1824) from the under-control controllers, gated on
        /// <see cref="SimShipController.ControlGyros"/>, and write the
        /// result into <see cref="GyroRateSystem.ControlTorque"/>.
        /// </summary>
        public void ComputeControlTorque()
        {
            _lastControlTorque = Vector3.Zero;
            _rotationActive = false;

            SimShipController source = null;
            for (int i = 0; i < _controllers.Count; i++)
            {
                SimShipController c = _controllers[i];
                if (c.Closed || !c.IsWorking || !c.IsUnderControl || !c.ControlGyros)
                {
                    continue;
                }
                source = c; // last under-control controller wins (assignment, not accumulation)
            }

            if (source == null)
            {
                _grid.GyroRateSystem.ControlTorque = Vector3.Zero;
                return;
            }
            _rotationActive = true;

            // MyShipController.MoveAndRotate (:1816-1824):
            //   vector2 = RotationIndicator / 20f; ClampToSphere 1
            //   num    = RollIndicator * 0.2f
            //   vector3 = Transform(new Vector3(-vector2.X, -vector2.Y, -num), blockOrientation)
            //   ClampToSphere(vector3, 1)
            //   m_gridGyroSystem.ControlTorque = vector3
            Vector2 r = source.RotationIndicator / 20f;
            r = Vector2.ClampToSphere(r, 1f);
            float roll = source.RollIndicator * 0.2f;
            Vector3 command = new Vector3(-r.X, -r.Y, -roll);
            Matrix blockToGrid = Matrix.CreateFromQuaternion(source.Template.Orientation);
            Vector3 vector3 = Vector3.TransformNormal(command, blockToGrid);
            vector3 = Vector3.ClampToSphere(vector3, 1f);

            _lastControlTorque = vector3;
            _grid.GyroRateSystem.ControlTorque = vector3;
        }
    }
}
