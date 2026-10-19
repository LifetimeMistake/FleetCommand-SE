using System.Collections.Generic;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>A remote control implementing <see cref="IMyRemoteControl"/>: extends
    /// <see cref="SimShipController"/> and adds <see cref="SpeedLimit"/>, the autopilot-enabled
    /// flag, and minimal waypoint storage (a list + current target) that the grid's
    /// <see cref="AutopilotSystem"/> flies. When autopilot is enabled the grid's gyros route to the
    /// non-override authority pool (<see cref="GyroRateSystem.AutopilotEnabled"/>) and the remote
    /// is treated as under control (mirroring the game's <c>SetAutoPilotEnabled</c> routing —
    /// <c>ThrustComp.AutopilotEnabled</c> → the <c>ComputeAiThrust</c> path,
    /// MyEntityThrustComponent.cs:1457-1478).</summary>
    public sealed class SimRemoteControl : SimShipController, IMyRemoteControl
    {
        private bool _isAutoPilotEnabled;
        private float _speedLimit;
        private readonly List<MyWaypointInfo> _waypoints = new List<MyWaypointInfo>();
        private int _waypointIndex;
        private FlightMode _flightMode = FlightMode.Patrol;

        public override float PowerDemand
        {
            get { return Template.RequiredPowerInput == 0f ? base.PowerDemand :
                (IsUnderControl || IsAutoPilotEnabled ? Template.RequiredPowerInput : 1e-7f); }
            set { base.PowerDemand = value; }
        }

        public SimRemoteControl(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
        }

        /// <inheritdoc />
        public bool IsAutoPilotEnabled => _isAutoPilotEnabled;

        /// <summary>The autopilot speed limit (m/s): the cap on the forward throttle. The game
        /// publishes it as <c>MovementBlockMaxSpeed</c> and gates the thrust on <c>|v|² &lt;
        /// maxSpeed²</c> (MyAutopilotComponent.cs:1322, :2805).</summary>
        public float SpeedLimit
        {
            get { return _speedLimit; }
            set { _speedLimit = value; }
        }

        /// <summary>Enables or disables the remote's autopilot. Enabling routes the grid's thrust
        /// to the AI command path and its gyros to the authority pool (the game's
        /// <c>MyAutopilotComponent.SetAutoPilotEnabled</c> :3009 →
        /// <c>ThrustComp.AutopilotEnabled</c>) and marks the remote under control; disabling coasts
        /// (no AI command).</summary>
        public void SetAutoPilotEnabled(bool enabled)
        {
            if (enabled && (!IsFunctional || _waypoints.Count == 0 || (_waypoints.Count == 1 && FlightMode != FlightMode.OneWay))) return;
            if (_isAutoPilotEnabled == enabled)
            {
                return;
            }
            _isAutoPilotEnabled = enabled;
            IsUnderControl = enabled;
            Grid.GyroRateSystem.AutopilotEnabled = enabled;
        }

        /// <inheritdoc />
        /// <summary>The current flight target waypoint, or <see cref="MyWaypointInfo.Empty"/> when
        /// none is set/reached.</summary>
        public MyWaypointInfo CurrentWaypoint =>
            _waypointIndex < _waypoints.Count ? _waypoints[_waypointIndex] : MyWaypointInfo.Empty;

        /// <inheritdoc />
        public void AddWaypoint(Vector3D coords, string name)
        {
            _waypoints.Add(new MyWaypointInfo(name, coords));
        }

        /// <inheritdoc />
        public void AddWaypoint(MyWaypointInfo coords)
        {
            _waypoints.Add(coords);
        }

        /// <inheritdoc />
        public void ClearWaypoints()
        {
            _waypoints.Clear();
            _waypointIndex = 0;
        }

        /// <inheritdoc />
        public void GetWaypointInfo(List<MyWaypointInfo> waypoints)
        {
            if (waypoints == null)
            {
                return;
            }
            waypoints.Clear();
            for (int i = 0; i < _waypoints.Count; i++)
            {
                waypoints.Add(_waypoints[i]);
            }
        }

        /// <summary>True while a target waypoint is set (for the
        /// <see cref="AutopilotSystem"/>).</summary>
        internal bool HasTarget => _waypointIndex < _waypoints.Count;

        /// <summary>Advances to the next waypoint (or stays on the last), called by the
        /// <see cref="AutopilotSystem"/> when the current target is reached.</summary>
        internal void AdvanceWaypoint()
        {
            if (_waypointIndex < _waypoints.Count - 1)
            {
                _waypointIndex++;
            }
        }

        // — waypoint / autopilot members outside the implemented contract —

        /// <inheritdoc />
        public FlightMode FlightMode
        {
            get { return _flightMode; }
            set { _flightMode = value; }
        }

        /// <inheritdoc />
        public Base6Directions.Direction Direction
        {
            get { return ThrowNotSupported<Base6Directions.Direction>("IMyRemoteControl.Direction"); }
            set { ThrowSetNotSupported("IMyRemoteControl.Direction"); }
        }

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
        public void SetCollisionAvoidance(bool enabled) => ThrowNotSupportedVoid("IMyRemoteControl.SetCollisionAvoidance()");

        /// <inheritdoc />
        public void SetDockingMode(bool enabled) => ThrowNotSupportedVoid("IMyRemoteControl.SetDockingMode()");
    }
}
