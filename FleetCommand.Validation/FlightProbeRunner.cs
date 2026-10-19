using System;
using System.Collections.Generic;
using System.Text;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Validation
{
    /// <summary>Shared, observable flight experiments. Only ingame services enter this class.</summary>
    public sealed class FlightProbeRunner : IValidationExperiment
    {
        public const string Header = "FCDIAGNOSTIC|1|craft-v1|flight-v1";
        public const string Columns = "COLUMNS|case|tick|seconds|dt|px|py|pz|vx|vy|vz|wx|wy|wz|qx|qy|qz|qw|thrust_px|thrust_nx|thrust_py|thrust_ny|thrust_pz|thrust_nz|battery_in|battery_out|battery_mwh|reactor_mw|reactor_a_mw|reactor_b_mw|reactor_c_mw|dampeners|autopilot|connector|distance|target_x|target_y|target_z|speed|reactor_enabled_mask|reactor_working_mask|battery_enabled|battery_working|thrust_enabled_mask|thrust_working_mask|gyro_working_mask";
        readonly string[] _names = { "power_full_rest", "power_limited_rest", "power_full_moving", "power_limited_moving", "waypoint_forward", "waypoint_turn" };
        readonly IMyGridProgramRuntimeInfo _runtime;
        readonly Action<string> _echo;
        readonly IMyRemoteControl _reference;
        readonly IMyCockpit _cockpit;
        readonly IMyBatteryBlock _battery;
        readonly IMyShipConnector _connector;
        readonly IMyTextPanel _lcd;
        readonly IMyReactor[] _reactors = new IMyReactor[3];
        readonly IMyThrust[] _thrust = new IMyThrust[6];
        readonly IMyGyro[] _gyros = new IMyGyro[2];
        readonly ValidationReport _report;
        readonly SettlingGate _gate = new SettlingGate();
        readonly StringBuilder _row = new StringBuilder();
        InitialFrame _frame;
        Vector3D _target;
        double _mass;
        int _case, _tick, _run, _settleTicks, _warmTicks, _samples, _zeroCallbacks;
        bool _settling, _warming, _firstInterval;
        bool _waitingForPublication;
        string _lastOutcome = "idle";
        public bool IsRunning { get; private set; }
        public string Report { get { return _report.Text; } }

        public FlightProbeRunner(IMyGridTerminalSystem gts, IMyGridProgramRuntimeInfo runtime,
            IMyProgrammableBlock me, Action<string> echo, string previousReport)
        {
            _runtime = runtime; _echo = echo;
            var blocks = new List<IMyTerminalBlock>(); gts.GetBlocks(blocks);
            for (int i = blocks.Count - 1; i >= 0; i--)
                if (blocks[i].CubeGrid.EntityId != me.CubeGrid.EntityId) blocks.RemoveAt(i);
            _reference = Find(blocks, "FC Reference") as IMyRemoteControl;
            _cockpit = Find(blocks, "FC Cockpit") as IMyCockpit;
            _battery = Find(blocks, "FC Battery") as IMyBatteryBlock;
            _connector = Find(blocks, "FC Connector") as IMyShipConnector;
            _reactors[0] = Find(blocks, "FC Reactor") as IMyReactor;
            _reactors[1] = Find(blocks, "FC Reactor B") as IMyReactor;
            _reactors[2] = Find(blocks, "FC Reactor C") as IMyReactor;
            string[] axes = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };
            for (int i = 0; i < 6; i++) _thrust[i] = Find(blocks, "FC Thrust " + axes[i]) as IMyThrust;
            _gyros[0] = Find(blocks, "FC Gyro A") as IMyGyro;
            _gyros[1] = Find(blocks, "FC Gyro B") as IMyGyro;
            _lcd = Find(blocks, "FC Report") as IMyTextPanel;
            _report = new ValidationReport(_lcd, previousReport);
            Cleanup(); _runtime.UpdateFrequency = UpdateFrequency.None;
            if (!string.IsNullOrEmpty(previousReport))
            {
                if (previousReport.IndexOf("END|", StringComparison.Ordinal) < 0) _report.Line("END|INTERRUPTED|RELOAD|0");
                _waitingForPublication = !_report.Publish(true);
                if (_waitingForPublication) _runtime.UpdateFrequency = UpdateFrequency.Update10;
            }
        }

        static IMyTerminalBlock Find(List<IMyTerminalBlock> blocks, string name)
        {
            IMyTerminalBlock result = null;
            for (int i = 0; i < blocks.Count; i++) if (blocks[i].CustomName == name)
            {
                if (result != null) throw new InvalidOperationException("Duplicate fixture name: " + name);
                result = blocks[i];
            }
            return result;
        }

        public void Main(string argument, UpdateType source)
        {
            try
            {
                string command = (argument ?? "").Trim().ToLowerInvariant();
                if (command == "flightprobe") { if (!IsRunning) Start(); return; }
                if (command == "abort") { Finish("ABORTED", "OPERATOR"); return; }
                if (command == "export") { _report.Publish(true); return; }
                if (command == "status") { _echo(IsRunning ? "Flight probe: " + _names[_case] + " | " + (_settling ? "settling" : _warming ? "warmup" : "tick " + _tick) : "Flight probe " + _lastOutcome); return; }
                if (!IsRunning)
                {
                    if (_waitingForPublication && (source & (UpdateType.Update1 | UpdateType.Update10)) != 0)
                    {
                        if (PublishPendingReport()) _runtime.UpdateFrequency = UpdateFrequency.None;
                        else _echo("Flight probe " + _lastOutcome + "; close FC Report editor to publish the buffered report.");
                    }
                    return;
                }
                if (command.Length != 0 || (source & UpdateType.Update1) == 0) return;
                double dt = _runtime.TimeSinceLastRun.TotalSeconds;
                if (dt == 0) { if (_firstInterval) _zeroCallbacks++; return; }
                _firstInterval = false;
                if (dt < 0.010 || dt > 0.025) { Finish("ERROR", "CADENCE_" + ValidationReport.Number(dt)); return; }
                _run++;
                if (_run > 18000) { Finish("ERROR", "RUN_TIMEOUT"); return; }
                if (_connector.Status == MyShipConnectorStatus.Connected) { Finish("ERROR", "CONNECTED_DURING_FLIGHT"); return; }
                if (_reference.GetNaturalGravity().Length() > 0.001) { Finish("ERROR", "GRAVITY_CHANGED"); return; }
                if (_cockpit.IsUnderControl || _reference.IsUnderControl && !_reference.IsAutoPilotEnabled)
                { Finish("ERROR", "PILOT_INPUT"); return; }
                if (_settling) Settle(dt); else if (_warming) Warmup(); else Measure(dt);
                if (_report.Overflow) { Finish("ERROR", "REPORT_OVERFLOW"); return; }
                _report.Publish(false); // An open editor blocks publication, not data collection.
                if (IsRunning && _run % 60 == 0) _echo("Flight probe " + _names[_case] + " | " + (_settling ? "settling" : _warming ? "warmup" : "tick " + _tick));
            }
            catch (Exception error) { Finish("ERROR", error.Message.Replace('|', '/').Replace('\n', ' ').Replace('\r', ' ')); }
        }

        void Start()
        {
            _report.Reset(); _report.Line(Header); _report.Line(Columns);
            _report.Line("PROTOCOL|power|net-+X-full-override|-X-off|balanced-orthogonal-load-pairs=+Y,-Y,+Z,-Z-full|dampeners-off|gyro-override-zero|full=three-reactors-and-battery-auto|limited=battery-discharge-only|180-frames");
            _report.Line("PROTOCOL|moving-control|full-power-wake-refresh-until-measured-thrust-direction-speed-at-least-1-mps|timeout-600-frames|then-switch-to-measurement-power");
            _report.Line("PROTOCOL|waypoint|forward-30m-and-right-behind-30m|speedlimit-3mps|OneWay|collision-avoidance-fixture-default|1500-frames-including-arrival-tail");
            _report.Line("SAMPLEPOLICY|power|first-12-then-every-60|waypoint|first-30-then-every-60|initial-and-final-always");
            _report.Line("FRAME|position-velocity-angular-attitude-target-vector|initial-reference-frame|target-vector=target-minus-current-reference-position");
            _report.Line("STATE|bitmasks|reactors=A-B-C|thrust=+X,-X,+Y,-Y,+Z,-Z|gyro=A-B|bit0=first");
            _report.Line("GATECOLUMNS|case|settle_ticks|passive_ticks|speed|angular_speed|passive_drift|passive_rotation");
            _report.Line("LIMITATION|records-raw-current-thrust-and-native-motion|no-power-shortage-normalization|arrival-is-observed-not-assumed|vacuum-zero-gravity-undocked-only");
            if (_reference == null || _cockpit == null || _battery == null || _connector == null || _lcd == null)
            { Finish("ERROR", "MISSING_REQUIRED_BLOCK"); return; }
            for (int i = 0; i < 3; i++) if (_reactors[i] == null) { Finish("ERROR", "MISSING_REACTOR"); return; }
            for (int i = 0; i < 6; i++) if (_thrust[i] == null) { Finish("ERROR", "MISSING_THRUSTER"); return; }
            for (int i = 0; i < 2; i++) if (_gyros[i] == null) { Finish("ERROR", "MISSING_GYRO"); return; }
            Cleanup();
            if (_connector.Status != MyShipConnectorStatus.Unconnected) { Finish("ERROR", "DISCONNECT_AND_MOVE_AWAY_FROM_DOCK"); return; }
            if (_reference.GetNaturalGravity().Length() > 0.001) { Finish("ERROR", "VACUUM_ZERO_GRAVITY_REQUIRED"); return; }
            if (_cockpit.IsUnderControl || _reference.IsUnderControl) { Finish("ERROR", "EXIT_COCKPIT_AND_REMOTE_CONTROL"); return; }
            if (_battery.CurrentStoredPower <= 0) { Finish("ERROR", "BATTERY_EMPTY"); return; }
            _mass = _reference.CalculateShipMass().PhysicalMass;
            if (_mass <= 0) { Finish("ERROR", "INVALID_MASS"); return; }
            _frame = new InitialFrame(_reference.WorldMatrix, _reference.CenterOfMass);
            var q = QuaternionD.CreateFromRotationMatrix(_reference.WorldMatrix.GetOrientation()); q.Normalize();
            if (q.W < 0) q = new QuaternionD(-q.X, -q.Y, -q.Z, -q.W);
            _report.Line("META|initial_qx|" + ValidationReport.RoundTripNumber(q.X));
            _report.Line("META|initial_qy|" + ValidationReport.RoundTripNumber(q.Y));
            _report.Line("META|initial_qz|" + ValidationReport.RoundTripNumber(q.Z));
            _report.Line("META|initial_qw|" + ValidationReport.RoundTripNumber(q.W));
            _report.Line("META|mass_kg|" + ValidationReport.Number(_mass));
            for (int i = 0; i < 6; i++)
            {
                if (_thrust[i].MaxEffectiveThrust <= 0 || Math.Abs(_thrust[i].MaxThrust - _thrust[i].MaxEffectiveThrust) > 1
                    || Vector3D.Distance(_frame.Vector(_thrust[i].WorldMatrix.Backward), Axis(i)) > 0.001)
                { Finish("ERROR", "INVALID_THRUSTER_" + i); return; }
                _report.Line("BLOCK|thrust|" + i + "|" + ValidationReport.Number(_thrust[i].MaxEffectiveThrust));
            }
            _case = 0; _run = 0; _zeroCallbacks = 0; _firstInterval = true;
            IsRunning = true; _runtime.UpdateFrequency = UpdateFrequency.Update1; BeginSettle();
            _report.Publish(true);
        }

        static Vector3D Axis(int i) { return i < 2 ? new Vector3D(i == 0 ? 1 : -1, 0, 0) : i < 4 ? new Vector3D(0, i == 2 ? 1 : -1, 0) : new Vector3D(0, 0, i == 4 ? 1 : -1); }

        void BeginSettle()
        {
            Cleanup(); _gate.Reset(); _settleTicks = 0; _settling = true; _warming = false;
        }

        void Settle(double dt)
        {
            _settleTicks++;
            var v = _reference.GetShipVelocities();
            var decision = _gate.Observe(_reference.CenterOfMass, _frame.Attitude(_reference.WorldMatrix), v.LinearVelocity,
                _cockpit.GetShipVelocities().LinearVelocity, v.AngularVelocity, dt);
            if (_settleTicks > 2400) { Finish("ERROR", "SETTLE_TIMEOUT_" + _names[_case]); return; }
            if (decision == SettleDecision.Brake)
            {
                Power(true); _reference.DampenersOverride = false;
                var local = Vector3D.TransformNormal(v.LinearVelocity, MatrixD.Transpose(_reference.WorldMatrix.GetOrientation()));
                for (int i = 0; i < 6; i++) _thrust[i].ThrustOverridePercentage = (float)Math.Min(1, Math.Max(0, -Vector3D.Dot(local, Axis(i)) * _mass * 2 / _thrust[i].MaxEffectiveThrust));
                ZeroGyros(true); return;
            }
            if (decision == SettleDecision.BeginCoast)
            {
                Cleanup(); for (int i = 0; i < 2; i++) _gyros[i].Enabled = false;
                return;
            }
            if (decision != SettleDecision.Ready) return;
            _report.Line("GATE|" + _names[_case] + "|" + _settleTicks + "|" + _gate.PassiveTicks + "|" + ValidationReport.Number(v.LinearVelocity.Length())
                + "|" + ValidationReport.Number(v.AngularVelocity.Length()) + "|" + ValidationReport.Number(_gate.PassiveDrift) + "|" + ValidationReport.Number(_gate.PassiveRotation));
            _settling = false; Cleanup(); _tick = 0; _samples = 0;
            if (_case == 2 || _case == 3)
            {
                _warming = true; _warmTicks = 0;
                _report.Line("WARMUP|" + _names[_case] + "|target_mps|1|timeout_frames|600"); ApplyWarmup();
            }
            else BeginMeasurement();
        }

        void ApplyWarmup()
        {
            Power(true); ZeroGyros(true);
            _reference.DampenersOverride = true; _reference.DampenersOverride = false;
            _thrust[0].ThrustOverridePercentage = 0; _thrust[0].ThrustOverridePercentage = 1;
        }

        void Warmup()
        {
            _warmTicks++;
            double speed = Vector3D.Dot(_reference.GetShipVelocities().LinearVelocity, _thrust[0].WorldMatrix.Backward);
            if (_warmTicks <= 12 || _warmTicks % 60 == 0 || speed >= 1)
                _report.Line("U|" + _names[_case] + "|" + _warmTicks + "|" + ValidationReport.Number(speed) + "|" + ValidationReport.Number(_thrust[0].CurrentThrust));
            if (speed >= 1)
            {
                _report.Line("WARMUPEND|" + _names[_case] + "|frames|" + _warmTicks + "|thrust_direction_mps|" + ValidationReport.Number(speed));
                _warming = false; BeginMeasurement(); return;
            }
            if (_warmTicks >= 600) { Finish("ERROR", "WARMUP_TIMEOUT_" + _names[_case]); return; }
            ApplyWarmup();
        }

        void BeginMeasurement()
        {
            _target = Vector3D.Zero;
            _report.Line("CASE|" + _names[_case] + "|" + (_case < 4 ? 180 : 1500));
            if (_case < 4)
            {
                Power(_case % 2 == 0); ZeroGyros(true);
                _thrust[0].ThrustOverridePercentage = 1; _reference.DampenersOverride = false;
                // Matched orthogonal pairs consume power but cancel transverse force.
                // The -X jet stays off, so the measured +X acceleration survives.
                for (int i = 2; i < 6; i++) _thrust[i].ThrustOverridePercentage = 1;
            }
            else
            {
                Power(true); ZeroGyros(false);
                var direction = _case == 4 ? _reference.WorldMatrix.Forward : Vector3D.Normalize(_reference.WorldMatrix.Right + _reference.WorldMatrix.Backward);
                _target = _reference.GetPosition() + direction * 30;
                _reference.ClearWaypoints(); _reference.AddWaypoint(_target, _names[_case]);
                _reference.FlightMode = FlightMode.OneWay; _reference.SpeedLimit = 3;
                _reference.SetAutoPilotEnabled(true);
            }
            Sample(0);
        }

        void Measure(double dt)
        {
            _tick++;
            int end = _case < 4 ? 180 : 1500;
            if (_tick <= (_case < 4 ? 12 : 30) || _tick % 60 == 0 || _tick == end) Sample(dt);
            if (_tick < end) return;
            _report.Line("ENDCASE|" + _names[_case] + "|" + _samples + "|autopilot|" + (_reference.IsAutoPilotEnabled ? 1 : 0));
            _case++;
            if (_case == _names.Length) { Finish("COMPLETE", "6"); return; }
            BeginSettle();
        }

        void Sample(double dt)
        {
            var velocities = _reference.GetShipVelocities();
            var p = _frame.Position(_reference.CenterOfMass); var v = _frame.Vector(velocities.LinearVelocity);
            var w = _frame.Vector(velocities.AngularVelocity); var q = _frame.Attitude(_reference.WorldMatrix);
            var target = _case >= 4 ? _frame.Vector(_target - _reference.GetPosition()) : Vector3D.Zero;
            _row.Clear(); _row.Append("S|").Append(_names[_case]).Append('|').Append(_tick);
            N(_run / 60.0); N(dt); V(p); V(v); V(w); N(q.X); N(q.Y); N(q.Z); N(q.W);
            for (int i = 0; i < 6; i++) N(_thrust[i].CurrentThrust);
            N(_battery.CurrentInput); N(_battery.CurrentOutput); N(_battery.CurrentStoredPower);
            double output = 0; int enabled = 0, working = 0;
            for (int i = 0; i < 3; i++) { output += _reactors[i].CurrentOutput; if (_reactors[i].Enabled) enabled |= 1 << i; if (_reactors[i].IsWorking) working |= 1 << i; }
            N(output); for (int i = 0; i < 3; i++) N(_reactors[i].CurrentOutput);
            N(_reference.DampenersOverride ? 1 : 0); N(_reference.IsAutoPilotEnabled ? 1 : 0); N((int)_connector.Status);
            N(target.Length()); V(target); N(velocities.LinearVelocity.Length()); N(enabled); N(working); N(_battery.Enabled ? 1 : 0); N(_battery.IsWorking ? 1 : 0);
            enabled = 0; working = 0;
            for (int i = 0; i < 6; i++) { if (_thrust[i].Enabled) enabled |= 1 << i; if (_thrust[i].IsWorking) working |= 1 << i; }
            N(enabled); N(working); N((_gyros[0].IsWorking ? 1 : 0) + (_gyros[1].IsWorking ? 2 : 0));
            _report.Line(_row.ToString()); _samples++;
        }

        void V(Vector3D v) { N(v.X); N(v.Y); N(v.Z); }
        void N(double n)
        {
            if (double.IsNaN(n) || double.IsInfinity(n)) throw new InvalidOperationException("NONFINITE_TELEMETRY");
            _row.Append('|').Append(ValidationReport.CompactNumber(n));
        }
        void Power(bool full)
        {
            for (int i = 0; i < 3; i++) if (_reactors[i] != null) _reactors[i].Enabled = full;
            if (_battery != null) { _battery.Enabled = true; _battery.ChargeMode = full ? ChargeMode.Auto : ChargeMode.Discharge; }
        }
        void ZeroGyros(bool overridden)
        {
            for (int i = 0; i < 2; i++) if (_gyros[i] != null)
            { _gyros[i].Enabled = true; _gyros[i].Pitch = 0; _gyros[i].Yaw = 0; _gyros[i].Roll = 0; _gyros[i].GyroPower = 1; _gyros[i].GyroOverride = overridden; }
        }
        void Cleanup()
        {
            if (_reference != null) { _reference.SetAutoPilotEnabled(false); _reference.ClearWaypoints(); _reference.DampenersOverride = false; }
            for (int i = 0; i < 6; i++) if (_thrust[i] != null) { _thrust[i].ThrustOverridePercentage = 0; _thrust[i].Enabled = true; }
            ZeroGyros(false); Power(true);
        }
        public bool PublishPendingReport()
        {
            bool published = _report.Publish(false);
            if (published) _waitingForPublication = false;
            return published;
        }

        void Finish(string status, string reason)
        {
            Cleanup(); IsRunning = false; _runtime.UpdateFrequency = UpdateFrequency.None;
            _lastOutcome = status + ": " + reason;
            _report.Line("END|" + status + "|" + reason + "|" + _run);
            _waitingForPublication = _lcd != null && !_report.Publish(true);
            if (_waitingForPublication) _runtime.UpdateFrequency = UpdateFrequency.Update10;
            _echo("Flight probe " + _lastOutcome);
        }
    }
}
