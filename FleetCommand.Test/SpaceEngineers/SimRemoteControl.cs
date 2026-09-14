using System.Collections.Generic;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// Remote control block, implementing the real <see cref="IMyRemoteControl"/>
    /// (from Sandbox.ModAPI.Ingame; , ). It extends
    /// <see cref="SimShipController"/> (so it provides the full
    /// ship-controller behavior and identity needed by
    /// discovery/configuration) and adds the scriptable remote surface this
    /// stage supports: <see cref="SpeedLimit"/> and the autopilot-enabled
    /// flag. The waypoint / autopilot methods serve no simulator behavior
    /// yet and throw a naming <see cref="NotSupportedException"/>.
    /// </summary>
    public sealed class SimRemoteControl : SimShipController, IMyRemoteControl
    {
        private bool _isAutoPilotEnabled;
        private float _speedLimit;

        public SimRemoteControl(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
        }

        /// <inheritdoc />
        public bool IsAutoPilotEnabled => _isAutoPilotEnabled;

        /// <summary>The autopilot speed limit (scenario-set; no live sector cap).</summary>
        public float SpeedLimit
        {
            get { return _speedLimit; }
            set { _speedLimit = value; }
        }

        /// <summary>Enables or disables the remote's autopilot flag.</summary>
        public void SetAutoPilotEnabled(bool enabled)
        {
            _isAutoPilotEnabled = enabled;
        }

        // — waypoint / autopilot members outside the  contract —

        /// <inheritdoc />
        public FlightMode FlightMode
        {
            get { return ThrowNotSupported<FlightMode>("IMyRemoteControl.FlightMode"); }
            set { ThrowSetNotSupported("IMyRemoteControl.FlightMode"); }
        }

        /// <inheritdoc />
        public Base6Directions.Direction Direction
        {
            get { return ThrowNotSupported<Base6Directions.Direction>("IMyRemoteControl.Direction"); }
            set { ThrowSetNotSupported("IMyRemoteControl.Direction"); }
        }

        /// <inheritdoc />
        public MyWaypointInfo CurrentWaypoint => ThrowNotSupported<MyWaypointInfo>("IMyRemoteControl.CurrentWaypoint");

        /// <inheritdoc />
        public bool WaitForFreeWay
        {
            get { return ThrowNotSupported<bool>("IMyRemoteControl.WaitForFreeWay"); }
            set { ThrowSetNotSupported("IMyRemoteControl.WaitForFreeWay"); }
        }

        /// <inheritdoc />
        public bool GetNearestPlayer(out Vector3D playerPosition)
        {
            playerPosition = Vector3D.Zero;
            throw NotSupported("IMyRemoteControl.GetNearestPlayer()");
        }

        /// <inheritdoc />
        public void ClearWaypoints() => ThrowNotSupportedVoid("IMyRemoteControl.ClearWaypoints()");

        /// <inheritdoc />
        public void GetWaypointInfo(List<MyWaypointInfo> waypoints) => ThrowNotSupportedVoid("IMyRemoteControl.GetWaypointInfo()");

        /// <inheritdoc />
        public void AddWaypoint(Vector3D coords, string name) => ThrowNotSupportedVoid("IMyRemoteControl.AddWaypoint()");

        /// <inheritdoc />
        public void AddWaypoint(MyWaypointInfo coords) => ThrowNotSupportedVoid("IMyRemoteControl.AddWaypoint()");

        /// <inheritdoc />
        public void SetCollisionAvoidance(bool enabled) => ThrowNotSupportedVoid("IMyRemoteControl.SetCollisionAvoidance()");

        /// <inheritdoc />
        public void SetDockingMode(bool enabled) => ThrowNotSupportedVoid("IMyRemoteControl.SetDockingMode()");
    }
}
