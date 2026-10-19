using System;
using System.Collections.Generic;
using System.Text;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Validation
{
    /// <summary>One-paste station mission. Navigation uses PB APIs and a recorded mating pose;
    /// it never teleports or takes native simulator shortcuts.</summary>
    public sealed class DockingProbeRunner : IValidationExperiment
    {
        public const string Header = "FCDIAGNOSTIC|1|station-v1|dock-v1";
        readonly IMyGridTerminalSystem _gts;
        readonly IMyGridProgramRuntimeInfo _runtime;
        readonly IMyProgrammableBlock _me;
        readonly Action<string> _echo;
        readonly IMyRemoteControl _reference;
        readonly IMyCockpit _cockpit;
        readonly IMyShipConnector _port;
        readonly IMyTextPanel _dockLcd;
        readonly ValidationReport _report;
        readonly List<IMyThrust> _thrusters = new List<IMyThrust>();
        readonly List<IMyGyro> _gyros = new List<IMyGyro>();
        readonly List<IMyReactor> _reactors = new List<IMyReactor>();
        readonly List<IMyBatteryBlock> _batteries = new List<IMyBatteryBlock>();
        IMyShipConnector _stationPort;
        FlightProbeRunner _flight;
        InitialFrame _frame;
        MatrixD _stationPose, _matingInStation;
        Vector3D _homeCom, _outward;
        int _phase, _phaseTick, _ticks, _stableTicks, _samples, _captureStableTicks;
        int _lastSampleTick = -1, _lastSamplePhase = -1;
        bool _connectableObserved;
        bool _waitingForPublication;
        string _lastOutcome = "idle";
        public bool IsRunning { get; private set; }
        public string Report { get { return _report.Text; } }
        static readonly string[] PhaseNames = { "connected_baseline", "release", "retreat", "standoff", "flight", "return", "capture", "lock", "locked_hold" };

        public DockingProbeRunner(IMyGridTerminalSystem gts, IMyGridProgramRuntimeInfo runtime,
            IMyProgrammableBlock me, Action<string> echo, string previousReport)
        {
            _gts = gts; _runtime = runtime; _me = me; _echo = echo;
            var blocks = new List<IMyTerminalBlock>(); gts.GetBlocks(blocks);
            foreach (var block in blocks)
            {
                if (block.CubeGrid.EntityId != me.CubeGrid.EntityId) continue;
                if (block.CustomName == "FC Reference") _reference = block as IMyRemoteControl;
                if (block.CustomName == "FC Cockpit") _cockpit = block as IMyCockpit;
                if (block.CustomName == "FC Connector") _port = block as IMyShipConnector;
                if (block.CustomName == "FC Dock Report") _dockLcd = block as IMyTextPanel;
                if (block is IMyThrust) _thrusters.Add((IMyThrust)block);
                if (block is IMyGyro) _gyros.Add((IMyGyro)block);
                if (block is IMyReactor) _reactors.Add((IMyReactor)block);
                if (block is IMyBatteryBlock) _batteries.Add((IMyBatteryBlock)block);
            }
            string[] thrustNames = { "FC Thrust +X", "FC Thrust -X", "FC Thrust +Y", "FC Thrust -Y", "FC Thrust +Z", "FC Thrust -Z" };
            _thrusters.Sort((a, b) => Array.IndexOf(thrustNames, a.CustomName).CompareTo(Array.IndexOf(thrustNames, b.CustomName)));
            _report = new ValidationReport(_dockLcd,
                previousReport != null && previousReport.StartsWith(Header) ? previousReport : "");
            // Overrides survive saves/reloads. Interrupting this mission must not
            // leave an idle replacement program flying the last command forever.
            if (_reference != null) { ResetFlight(); _reference.DampenersOverride = true; }
            _runtime.UpdateFrequency = UpdateFrequency.None;
            if (_report.Text.Length > 0)
            {
                if (_report.Text.IndexOf("END|", StringComparison.Ordinal) < 0) _report.Line("END|INTERRUPTED|RELOAD");
                _waitingForPublication = !_report.Publish(true);
                if (_waitingForPublication) _runtime.UpdateFrequency = UpdateFrequency.Update10;
            }
        }

        public void Main(string argument, UpdateType source)
        {
            try
            {
                string command = (argument ?? "").Trim().ToLowerInvariant();
                if (command == "abort") { if (_flight != null && _flight.IsRunning) _flight.Main("abort", source); Finish("ABORTED", "OPERATOR"); return; }
                if (command == "status" || command == "export")
                {
                    _report.Publish(true); _echo(StatusText());
                    if (_phase == 4 && _flight != null) _flight.Main("status", source);
                    return;
                }
                if (command == "dockprobe") { if (!IsRunning) Start(); return; }
                if (!IsRunning)
                {
                    if (_waitingForPublication && (source & (UpdateType.Update1 | UpdateType.Update10)) != 0)
                    {
                        if (PublishReports()) _runtime.UpdateFrequency = UpdateFrequency.None;
                        else _echo(StatusText() + "; close the LCD editors to publish buffered reports.");
                    }
                    return;
                }
                if ((source & UpdateType.Update1) == 0) return;
                double dt = _runtime.TimeSinceLastRun.TotalSeconds;
                if (dt == 0) return;
                if (dt < 0.01 || dt > 0.025) { Finish("ERROR", "CADENCE_" + ValidationReport.CompactNumber(dt)); return; }
                if (_cockpit != null && _cockpit.IsUnderControl || _reference.IsUnderControl && !_reference.IsAutoPilotEnabled)
                { Finish("ERROR", "PILOT_INPUT"); return; }
                if (_reference.GetNaturalGravity().Length() > 0.001) { Finish("ERROR", "GRAVITY"); return; }
                if (Vector3D.Distance(_stationPose.Translation, _stationPort.WorldMatrix.Translation) > 0.001)
                { Finish("ERROR", "STATION_MOVED"); return; }
                _ticks++; _phaseTick++;
                Tick(dt);
                if (!IsRunning) return;
                if (_phase != 4 && (_phaseTick <= 12 || _phaseTick % 30 == 0)) Sample();
                if (_phaseTick % 60 == 0)
                {
                    PublishReports();
                    if (_phase != 4) _echo(StatusText());
                }
                if (_report.Overflow) Finish("ERROR", "REPORT_OVERFLOW");
            }
            catch (Exception e) { Finish("ERROR", e.GetType().Name + ":" + e.Message); }
        }

        void Start()
        {
            _report.Reset(); _report.Line(Header); _ticks = 0; _samples = 0;
            _lastSampleTick = -1; _lastSamplePhase = -1; _frame = null; _stationPort = null; _flight = null;
            _report.Line("ARTIFACT|docking|FC Dock Report");
            _report.Line("ARTIFACT|flight|FC Report");
            _report.Line("PROTOCOL|static-station|initially-connected|retreat-m|40|flightprobe|return-to-recorded-mating-transform|explicit-connect");
            _report.Line("LIMITATION|cross-grid-power-and-terminal-discovery-are-not-validated-by-this-mission");
            if (_reference == null || _port == null || _dockLcd == null || _thrusters.Count != 6 || _gyros.Count != 2)
            { Finish("ERROR", "USE_STATION_FIXTURE"); return; }
            if (_port.Status != MyShipConnectorStatus.Connected || _port.OtherConnector == null)
            { Finish("ERROR", "START_CONNECTED"); return; }
            _stationPort = _port.OtherConnector;
            if (!_stationPort.CubeGrid.IsStatic || _me.CubeGrid.IsStatic)
            { Finish("ERROR", "REQUIRES_STATIC_STATION_AND_DYNAMIC_CRAFT"); return; }
            if (_reference.GetShipSpeed() > 0.05 || _reference.GetShipVelocities().AngularVelocity.Length() > 0.01)
            { Finish("ERROR", "START_MOVING"); return; }
            _stationPose = _stationPort.WorldMatrix;
            _matingInStation = _port.WorldMatrix * MatrixD.Invert(_stationPose);
            _homeCom = _reference.CenterOfMass;
            _outward = Vector3D.Normalize(_port.WorldMatrix.Translation - _stationPose.Translation);
            _frame = new InitialFrame(_reference.WorldMatrix, _homeCom);
            var q = QuaternionD.CreateFromRotationMatrix(_reference.WorldMatrix.GetOrientation()); q.Normalize();
            _report.Line("META|initial_qx|" + ValidationReport.RoundTripNumber(q.X));
            _report.Line("META|initial_qy|" + ValidationReport.RoundTripNumber(q.Y));
            _report.Line("META|initial_qz|" + ValidationReport.RoundTripNumber(q.Z));
            _report.Line("META|initial_qw|" + ValidationReport.RoundTripNumber(q.W));
            _report.Line("META|mass_kg|" + ValidationReport.Number(_reference.CalculateShipMass().PhysicalMass));
            _report.Line("COLUMNS|phase|tick|phase_tick|px|py|pz|vx|vy|vz|wx|wy|wz|qx|qy|qz|qw|mating_error_m|port|station_port|connector_distance_m|thrust_px|thrust_nx|thrust_py|thrust_ny|thrust_pz|thrust_nz");
            ResetFlight(); _reference.DampenersOverride = false;
            _connectableObserved = false; _captureStableTicks = 0; _flight = null; IsRunning = true;
            _runtime.UpdateFrequency = UpdateFrequency.Update1;
            Enter(0);
            _report.Publish(true);
        }

        void Tick(double dt)
        {
            if (_phase == 0)
            {
                ZeroThrust();
                if (_phaseTick >= 60)
                {
                    _port.Disconnect(); _port.Enabled = false; _stationPort.Enabled = false;
                    Enter(1);
                }
            }
            else if (_phase == 1)
            {
                if (_port.Status != MyShipConnectorStatus.Connected && _stationPort.Status != MyShipConnectorStatus.Connected)
                { _report.Line("CHECK|release|PASS"); Enter(2); }
                else if (_phaseTick > 120) Finish("ERROR", "RELEASE_TIMEOUT");
            }
            else if (_phase == 2 || _phase == 3)
            {
                Vector3D target = _homeCom + _outward * 40;
                Navigate(target, 1.5);
                bool settled = Vector3D.Distance(_reference.CenterOfMass, target) < 0.2 && _reference.GetShipSpeed() < 0.05;
                _stableTicks = settled ? _stableTicks + 1 : 0;
                if (_phase == 2 && _stableTicks >= 30) Enter(3);
                else if (_phase == 3 && _stableTicks >= 120)
                {
                    ZeroThrust();
                    _flight = new FlightProbeRunner(_gts, _runtime, _me, _echo, "");
                    _flight.Main("flightprobe", UpdateType.Terminal);
                    if (!_flight.IsRunning) { Finish("ERROR", "FLIGHT_START_FAILED"); return; }
                    Enter(4);
                }
                if (_phaseTick > 6000) Finish("ERROR", "RETREAT_TIMEOUT");
            }
            else if (_phase == 4)
            {
                _flight.Main("", UpdateType.Update1);
                if (!_flight.IsRunning)
                {
                    if (_flight.Report.IndexOf("END|COMPLETE|", StringComparison.Ordinal) < 0)
                    { Finish("ERROR", "FLIGHT_FAILED_SEE_FC_REPORT"); return; }
                    _report.Line("CHECK|flight_report_complete|PASS|characters|" + _flight.Report.Length);
                    ResetFlight(); _reference.DampenersOverride = false;
                    _runtime.UpdateFrequency = UpdateFrequency.Update1;
                    Enter(5);
                }
            }
            else if (_phase == 5)
            {
                // Return the craft to the captured docking attitude before the final approach.
                MatrixD desiredPort = _matingInStation * _stationPort.WorldMatrix;
                Vector3D rotation = OrientationError(_port.WorldMatrix, desiredPort);
                SetGyroRate(rotation * 0.8);
                Vector3D targetPort = desiredPort.Translation + _outward * 1.5;
                Navigate(_reference.CenterOfMass + targetPort - _port.WorldMatrix.Translation, 1.5);
                bool ready = Vector3D.Distance(_port.WorldMatrix.Translation, targetPort) < 0.15
                    && _reference.GetShipSpeed() < 0.05 && rotation.Length() < 0.01;
                _stableTicks = ready ? _stableTicks + 1 : 0;
                if (_stableTicks >= 60) { _port.Enabled = true; _stationPort.Enabled = true; Enter(6); }
                else if (_phaseTick > 9000) Finish("ERROR", "RETURN_TIMEOUT");
            }
            else if (_phase == 6)
            {
                MatrixD desiredPort = _matingInStation * _stationPort.WorldMatrix;
                SetGyroRate(OrientationError(_port.WorldMatrix, desiredPort) * 0.8);
                Navigate(_reference.CenterOfMass + desiredPort.Translation - _port.WorldMatrix.Translation, 0.5);
                if (_port.Status == MyShipConnectorStatus.Connected)
                { Finish("ERROR", "LOCKED_WITHOUT_CONNECT_COMMAND"); return; }
                if (_port.Status == MyShipConnectorStatus.Connectable)
                {
                    ZeroThrust();
                    if (!_connectableObserved) { _connectableObserved = true; _report.Line("CHECK|capture_without_lock|PASS"); }
                    _captureStableTicks++;
                    if (_captureStableTicks >= 60) { _port.Connect(); _report.Line("COMMAND|connect|" + _ticks); Enter(7); }
                }
                else
                {
                    _captureStableTicks = 0;
                    if (_phaseTick > 1800) Finish("ERROR", "CAPTURE_TIMEOUT");
                }
            }
            else if (_phase == 7)
            {
                ZeroThrust();
                if (_port.Status == MyShipConnectorStatus.Connected && _stationPort.Status == MyShipConnectorStatus.Connected)
                { _report.Line("CHECK|explicit_lock|PASS"); Enter(8); }
                else if (_phaseTick > 300) Finish("ERROR", "LOCK_TIMEOUT");
            }
            else if (_phase == 8)
            {
                ZeroThrust(); SetGyroRate(Vector3D.Zero);
                if (_port.Status != MyShipConnectorStatus.Connected) { Finish("ERROR", "LOCK_LOST"); return; }
                if (_phaseTick >= 120) Finish("COMPLETE", "9");
            }
        }

        void Enter(int phase)
        {
            _phase = phase; _phaseTick = 0; _stableTicks = 0;
            _report.Line("PHASE|" + PhaseNames[phase] + "|" + _ticks);
            Sample(); _report.Publish(false); _echo("Dock mission: " + PhaseNames[phase]);
        }

        void Navigate(Vector3D target, double maxSpeed)
        {
            Vector3D error = target - _reference.CenterOfMass;
            Vector3D desired = error * 0.6;
            if (desired.Length() > maxSpeed) desired = Vector3D.Normalize(desired) * maxSpeed;
            Vector3D acceleration = (desired - _reference.GetShipVelocities().LinearVelocity) * 3;
            double mass = _reference.CalculateShipMass().PhysicalMass;
            foreach (var thrust in _thrusters)
            {
                double force = Vector3D.Dot(acceleration, thrust.WorldMatrix.Backward) * mass;
                thrust.ThrustOverridePercentage = (float)Math.Max(0, Math.Min(1, force / Math.Max(1, thrust.MaxEffectiveThrust)));
            }
        }

        static Vector3D OrientationError(MatrixD current, MatrixD target)
        {
            MatrixD delta = current.GetOrientation() * MatrixD.Transpose(target.GetOrientation());
            var q = QuaternionD.CreateFromRotationMatrix(delta); q.Normalize();
            if (q.W < 0) q = new QuaternionD(-q.X, -q.Y, -q.Z, -q.W);
            Vector3D local = new Vector3D(q.X, q.Y, q.Z);
            double length = local.Length();
            if (length < 1e-9) return Vector3D.Zero;
            // delta maps desired axes to current; negate its axis to turn back.
            return Vector3D.TransformNormal(local * (-2 * Math.Atan2(length, q.W) / length), target.GetOrientation());
        }

        void SetGyroRate(Vector3D worldRate)
        {
            if (worldRate.Length() > 0.3) worldRate = Vector3D.Normalize(worldRate) * 0.3;
            foreach (var gyro in _gyros)
            {
                var local = Vector3D.TransformNormal(worldRate, MatrixD.Transpose(gyro.WorldMatrix.GetOrientation()));
                gyro.Enabled = true; gyro.GyroOverride = true;
                gyro.Pitch = (float)-local.X; gyro.Yaw = (float)-local.Y; gyro.Roll = (float)-local.Z;
            }
        }

        void ZeroThrust() { foreach (var thrust in _thrusters) thrust.ThrustOverridePercentage = 0; }
        void ResetFlight()
        {
            ZeroThrust(); _reference.SetAutoPilotEnabled(false); _reference.ClearWaypoints();
            foreach (var reactor in _reactors) reactor.Enabled = true;
            foreach (var battery in _batteries) battery.ChargeMode = ChargeMode.Auto;
            foreach (var gyro in _gyros) { gyro.Enabled = true; gyro.GyroOverride = false; }
        }

        void Sample()
        {
            if (_frame == null || _stationPort == null) return;
            if (_lastSampleTick == _ticks && _lastSamplePhase == _phase) return;
            _lastSampleTick = _ticks; _lastSamplePhase = _phase;
            var p = _frame.Position(_reference.CenterOfMass);
            var velocity = _reference.GetShipVelocities();
            var v = _frame.Vector(velocity.LinearVelocity); var w = _frame.Vector(velocity.AngularVelocity);
            var q = _frame.Attitude(_reference.WorldMatrix);
            var row = new StringBuilder("S|").Append(PhaseNames[_phase]).Append('|').Append(_ticks).Append('|').Append(_phaseTick);
            double[] values = { p.X,p.Y,p.Z,v.X,v.Y,v.Z,w.X,w.Y,w.Z,q.X,q.Y,q.Z,q.W,
                Vector3D.Distance(_port.WorldMatrix.Translation, (_matingInStation * _stationPort.WorldMatrix).Translation),
                (int)_port.Status,(int)_stationPort.Status,Vector3D.Distance(_port.WorldMatrix.Translation,_stationPort.WorldMatrix.Translation) };
            foreach (double value in values) row.Append('|').Append(ValidationReport.CompactNumber(value));
            foreach (var thrust in _thrusters) row.Append('|').Append(ValidationReport.CompactNumber(thrust.CurrentThrust));
            _report.Line(row.ToString()); _samples++;
        }

        void Finish(string status, string reason)
        {
            if (IsRunning) Sample();
            if (_flight != null && _flight.IsRunning) _flight.Main("abort", UpdateType.Terminal);
            if (_reference != null) { ResetFlight(); _reference.DampenersOverride = true; }
            if (_port != null) _port.Enabled = true;
            if (_stationPort != null) _stationPort.Enabled = true;
            IsRunning = false; _lastOutcome = status + ": " + reason;
            _report.Line("END|" + status + "|" + reason.Replace('|', '/').Replace('\n', ' ').Replace('\r', ' ') + "|" + _samples);
            _runtime.UpdateFrequency = PublishReports() ? UpdateFrequency.None : UpdateFrequency.Update10;
            _echo(StatusText() + ". Copy FC Dock Report and FC Report.");
        }

        bool PublishReports()
        {
            bool docking = _report.Publish(false);
            bool flight = _flight == null || _flight.IsRunning || _flight.PublishPendingReport();
            _waitingForPublication = !docking || !flight;
            return !_waitingForPublication;
        }

        string StatusText()
        {
            if (!IsRunning) return "Dock mission " + _lastOutcome;
            string status = "Dock mission " + PhaseNames[_phase] + " | phase tick " + _phaseTick
                + " | speed " + ValidationReport.CompactNumber(_reference.GetShipSpeed()) + " m/s";
            if (_phase == 2 || _phase == 3)
                status += " | standoff error " + ValidationReport.CompactNumber(Vector3D.Distance(_reference.CenterOfMass, _homeCom + _outward * 40)) + " m";
            return status;
        }
    }
}
