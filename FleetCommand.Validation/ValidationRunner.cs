using System;
using System.Collections.Generic;
using System.Text;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Validation
{
    /// <summary>All test behavior lives here. Both hosts supply only ingame services.</summary>
    public sealed class ValidationRunner
    {
        public const string Header = "FCVALIDATION|1|craft-v1|suite-v3";
        public const string GateColumns = "GATECOLUMNS|case|settle_tick|stable_ticks|passive_ticks|reference_speed|cockpit_speed|position_speed|reference_angular_speed|orientation_speed|passive_drift|passive_rotation";
        public const string GatePolicy = "GATEPOLICY|brake_frames|15|passive_frames|60|speed_mps|0.010000|angular_rps|0.010000|drift_m|0.010000|rotation_rad|0.010000|passive_actuators|zero-thrust-disabled-gyros";
        public const string Columns = "COLUMNS|case|tick|seconds|dt|px|py|pz|vx|vy|vz|wx|wy|wz|qx|qy|qz|qw|thrust_px|thrust_nx|thrust_py|thrust_ny|thrust_pz|thrust_nz|battery_in|battery_out|battery_delta_mwh|reactor_mw|dampeners|autopilot|connector|waypoint_m";
        readonly IMyGridProgramRuntimeInfo _runtime;
        readonly Action<string> _echo;
        readonly IMyRemoteControl _reference;
        readonly IMyCockpit _cockpit;
        readonly IMyTextPanel _lcd;
        readonly IMyBatteryBlock _battery;
        readonly IMyReactor _reactor;
        readonly IMyReactor _reactorB;
        readonly IMyReactor _reactorC;
        readonly IMyShipConnector _connector;
        readonly IMyThrust[] _thrusters = new IMyThrust[6];
        readonly IMyGyro[] _gyros = new IMyGyro[2];
        readonly ValidationCase[] _cases = ValidationCase.CreateSuite();
        readonly ValidationReport _report;
        readonly ValidationTrace _trace;
        readonly SettlingGate _settleGate = new SettlingGate();
        readonly List<IMyTerminalBlock> _blocks = new List<IMyTerminalBlock>();
        readonly StringBuilder _row = new StringBuilder();
        InitialFrame _frame;
        Vector3D _waypoint;
        int _caseIndex, _caseTick, _runTicks, _settleTicks;
        bool _passiveSettling;
        int _caseSamples;
        int _lcdProbeTicks = -1;
        bool _awaitingFirstInterval;
        int _startupZeroCallbacks;
        bool _settling;
        bool _lcdBlocked;
        double _mass;
        double _initialBattery;
        public bool IsRunning { get; private set; }
        public string Report { get { return _report.Text; } }

        public ValidationRunner(IMyGridTerminalSystem gts, IMyGridProgramRuntimeInfo runtime,
            IMyProgrammableBlock me, Action<string> echo, string previousReport)
        {
            _runtime = runtime; _echo = echo;
            gts.GetBlocks(_blocks);
            for (int i = _blocks.Count - 1; i >= 0; i--)
                if (_blocks[i].CubeGrid.EntityId != me.CubeGrid.EntityId) _blocks.RemoveAt(i);
            _reference = Find("FC Reference") as IMyRemoteControl;
            _cockpit = Find("FC Cockpit") as IMyCockpit;
            _lcd = Find("FC Report") as IMyTextPanel;
            _battery = Find("FC Battery") as IMyBatteryBlock;
            _reactor = Find("FC Reactor") as IMyReactor;
            _reactorB = Find("FC Reactor B") as IMyReactor;
            _reactorC = Find("FC Reactor C") as IMyReactor;
            _connector = Find("FC Connector") as IMyShipConnector;
            string[] directions = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };
            for (int i = 0; i < 6; i++) _thrusters[i] = Find("FC Thrust " + directions[i]) as IMyThrust;
            _gyros[0] = Find("FC Gyro A") as IMyGyro;
            _gyros[1] = Find("FC Gyro B") as IMyGyro;
            _report = new ValidationReport(_lcd, previousReport);
            _trace = new ValidationTrace(_report, _reference, _cockpit, me, _thrusters, _gyros);
            Cleanup();
            _runtime.UpdateFrequency = UpdateFrequency.None;
            if (!string.IsNullOrEmpty(previousReport))
            {
                if (previousReport.IndexOf("END|", StringComparison.Ordinal) < 0)
                    _report.Line("END|INTERRUPTED|RELOAD");
                _report.Publish(true);
            }
            _echo("FC validation ready. Commands: start, status, abort, export, dock.");
        }

        IMyTerminalBlock Find(string name)
        {
            IMyTerminalBlock found = null;
            for (int i = 0; i < _blocks.Count; i++)
                if (_blocks[i].CustomName == name)
                {
                    if (found != null) throw new InvalidOperationException("Duplicate fixture name: " + name);
                    found = _blocks[i];
                }
            return found;
        }

        public void Main(string argument, UpdateType source)
        {
            try
            {
                string command = (argument ?? "").Trim().ToLowerInvariant();
                if (command == "abort") { Finish("ABORTED", "OPERATOR"); return; }
                if (command == "export") { _report.Publish(true); _echo("Complete report is in FC Report LCD. " + Report.Length + " characters."); return; }
                if (command == "status") { _echo(IsRunning ? "Running " + _cases[_caseIndex].Name : "Idle. Report: " + Report.Length + " characters."); return; }
                if (command == "start") { if (!IsRunning) Start(); return; }
                if (command == "dock") { if (!IsRunning) Dock(); return; }
                if (command.Length != 0) { _echo("Unknown command: " + command); return; }
                if (!IsRunning || (source & UpdateType.Update1) == 0) return;
                double dt = _runtime.TimeSinceLastRun.TotalSeconds;
                // Terminal start and the first Update1 can share a game frame.
                // No physics elapsed: don't count it as a suite tick or a cadence failure.
                if (_awaitingFirstInterval && dt == 0 && _startupZeroCallbacks < 10)
                { _startupZeroCallbacks++; return; }
                if (dt < 0.010 || dt > 0.025) { Finish("ERROR", "CADENCE_" + ValidationReport.Number(dt)); return; }
                if (_awaitingFirstInterval)
                {
                    _awaitingFirstInterval = false;
                    if (_startupZeroCallbacks > 0)
                        _report.Line("INFO|startup_zero_elapsed_callbacks|" + _startupZeroCallbacks);
                }
                if (_lcdProbeTicks >= 0)
                {
                    _lcdProbeTicks++;
                    if (_lcd.GetText() != "FC-LCD-PROBE-APPEND")
                    {
                        if (_lcdProbeTicks > 120) Finish("ERROR", "LCD_ROUNDTRIP_TIMEOUT");
                        return;
                    }
                    _lcdProbeTicks = -1;
                    _report.Line("CHECK|lcd_append_read|PASS");
                    _report.Line(Columns);
                    _report.Line(GateColumns); _report.Line(GatePolicy); _report.Line(ValidationTrace.Columns);
                    _lcdBlocked = !_report.Publish(true);
                    return;
                }
                _runTicks++;
                if (_lcdBlocked) { Finish("ERROR", "LCD_WRITE_BLOCKED"); return; }
                if (_runTicks > 24000) { Finish("ERROR", "RUN_TIMEOUT"); return; }
                if (_reference.GetNaturalGravity().Length() > 0.001) { Finish("ERROR", "GRAVITY_CHANGED"); return; }
                if (_cockpit.IsUnderControl || _reference.IsUnderControl && !_reference.IsAutoPilotEnabled)
                { Finish("ERROR", "PILOT_INPUT"); return; }
                if (_settling) TickSettle(dt); else TickCase(dt);
                if (IsRunning)
                    _echo((_settling ? (_passiveSettling ? "PASSIVE REST before " : "BRAKING before ") : "MEASURING ")
                        + _cases[_caseIndex].Name + " | tick " + (_settling ? _settleTicks : _caseTick));
                if (_report.Overflow) { Cleanup(); IsRunning = false; _runtime.UpdateFrequency = UpdateFrequency.None; }
                _lcdBlocked = !_report.Publish(false);
            }
            catch (Exception error)
            {
                Finish("ERROR", error.Message.Replace('|', '/').Replace('\n', ' ').Replace('\r', ' '));
            }
        }

        void Start()
        {
            _report.Reset();
            _trace.Reset(null);
            _report.Line(Header);
            _report.Line("COVERAGE|automated|discovery,lcd,thrust,coast,gyro,rotated-gyro,damping,power,battery,waypoint,unpaired-connector");
            _report.Line("COVERAGE|manual|paired-docking:load-dock-align-then-run-dock,pilot-input,save-reload");
            _report.Line("COVERAGE|uncovered|planetary-gravity,atmosphere,collisions,fuel-depletion,speed-cap,all-block-variants,IGC,instruction-budget");
            _report.Line("COVERAGE|limitations|waypoint-steering-and-docking-approach-require-measured-review");
            if (_reference == null || _cockpit == null || _lcd == null || _battery == null || _reactor == null || _reactorB == null || _reactorC == null || _connector == null)
            { Finish("ERROR", "MISSING_REQUIRED_BLOCK"); return; }
            for (int i = 0; i < 6; i++) if (_thrusters[i] == null) { Finish("ERROR", "MISSING_THRUSTER"); return; }
            for (int i = 0; i < 2; i++) if (_gyros[i] == null) { Finish("ERROR", "MISSING_GYRO"); return; }
            Cleanup();
            var velocity = _reference.GetShipVelocities();
            if (velocity.LinearVelocity.Length() > 0.02 || velocity.AngularVelocity.Length() > 0.01)
            { Finish("ERROR", "START_AT_REST"); return; }
            if (_reference.GetNaturalGravity().Length() > 0.001)
            { Finish("ERROR", "VACUUM_ZERO_GRAVITY_REQUIRED"); return; }
            if (_cockpit.IsUnderControl || _reference.IsUnderControl)
            { Finish("ERROR", "EXIT_COCKPIT_AND_REMOTE_CONTROL"); return; }
            if (_connector.Status != MyShipConnectorStatus.Unconnected)
            { Finish("ERROR", "DISCONNECT_AND_MOVE_AWAY_FROM_DOCK"); return; }
            if (_battery.CurrentStoredPower < 0.02 || _battery.CurrentStoredPower > 0.04)
            { Finish("ERROR", "BATTERY_MUST_BE_0.02_TO_0.04_MWH"); return; }
            _mass = _reference.CalculateShipMass().PhysicalMass;
            _initialBattery = _battery.CurrentStoredPower;
            if (!_reactor.IsWorking || !_reactorB.IsWorking || !_reactorC.IsWorking || !_gyros[0].IsWorking || !_gyros[1].IsWorking)
            { Finish("ERROR", "CHECK_REACTOR_FUEL_AND_GYRO_POWER"); return; }
            _frame = new InitialFrame(_reference.WorldMatrix, _reference.CenterOfMass);
            _trace.Reset(_frame);
            _report.Line("META|mass_kg|" + ValidationReport.Number(_mass));
            _report.Line("META|battery_start_mwh|" + ValidationReport.Number(_battery.CurrentStoredPower));
            _report.Line("META|grid_size_m|" + ValidationReport.Number(_reference.CubeGrid.GridSize));
            _report.Line("META|terminal_blocks|" + _blocks.Count);
            for (int i = 0; i < 6; i++)
            {
                var t = _thrusters[i];
                if (!t.IsWorking || t.MaxThrust <= 0 || Math.Abs(t.MaxThrust - t.MaxEffectiveThrust) > 1)
                { Finish("ERROR", "UNPOWERED_OR_NONVACUUM_THRUSTER"); return; }
                Vector3D direction = _frame.Vector(t.WorldMatrix.Backward);
                var expected = Axis(i);
                if (Vector3D.Distance(direction, expected) > 0.001)
                { Finish("ERROR", "THRUSTER_ORIENTATION_" + i); return; }
                _report.Line("BLOCK|thrust|" + i + "|" + ValidationReport.Number(t.MaxThrust));
            }
            // Game text writes publish during UpdateAfterSimulation10. Keep the
            // probe intact and wait in subsequent Main calls before reading it.
            if (!_lcd.WriteText("FC-LCD-PROBE") || !_lcd.WriteText("-APPEND", true))
            { Finish("ERROR", "LCD_ROUNDTRIP"); return; }
            _caseIndex = 0; _runTicks = 0; _settleTicks = 0; _passiveSettling = false;
            _settleGate.Reset(); _trace.BeginSettle(_cases[0].Name);
            _settling = true; IsRunning = true; _lcdBlocked = false; _lcdProbeTicks = 0;
            _awaitingFirstInterval = true; _startupZeroCallbacks = 0;
            _runtime.UpdateFrequency = UpdateFrequency.Update1;
            _echo("Validation started. Leave craft untouched; all measurements go to FC Report.");
        }

        static Vector3D Axis(int index)
        {
            double sign = index % 2 == 0 ? 1 : -1;
            return index < 2 ? new Vector3D(sign, 0, 0) : index < 4 ? new Vector3D(0, sign, 0) : new Vector3D(0, 0, sign);
        }

        void TickSettle(double dt)
        {
            _settleTicks++;
            var velocity = _reference.GetShipVelocities();
            var cockpitVelocity = _cockpit.GetShipVelocities();
            var decision = _settleGate.Observe(_reference.CenterOfMass, _frame.Attitude(_reference.WorldMatrix),
                velocity.LinearVelocity, cockpitVelocity.LinearVelocity, velocity.AngularVelocity, dt);
            if (_settleTicks > 900) { Finish("ERROR", "SETTLE_TIMEOUT_" + _cases[_caseIndex].Name); return; }
            if (decision == SettleDecision.Brake)
            {
                if (_passiveSettling) _trace.Observe("passive_failed", _runTicks, _settleTicks, dt, 0);
                _passiveSettling = false;
                _trace.Observe("brake", _runTicks, _settleTicks, dt, _settleGate.StableTicks);
                Brake(velocity.LinearVelocity);
                return;
            }
            if (decision == SettleDecision.BeginCoast)
            {
                _trace.Observe("brake", _runTicks, _settleTicks, dt, _settleGate.StableTicks);
                _trace.FlushBrakeTail(); ResetCommands(); _passiveSettling = true;
                // A genuinely unactuated observation window: ordinary gyros also damp
                // rotation, so disable them until BeginCase restores normal settings.
                for (int i = 0; i < 2; i++) _gyros[i].Enabled = false;
                _echo("Checking passive rest before " + _cases[_caseIndex].Name);
                return;
            }
            if (_settleGate.PassiveTicks == 1 || _settleGate.PassiveTicks % 10 == 0)
                _trace.Observe("passive", _runTicks, _settleTicks, dt, _settleGate.StableTicks);
            if (decision != SettleDecision.Ready) return;
            _report.Line("SETTLE|" + _cases[_caseIndex].Name + "|" + _settleTicks);
            _row.Clear(); _row.Append("GATE|").Append(_cases[_caseIndex].Name);
            Number(_settleTicks); Number(_settleGate.StableTicks); Number(_settleGate.PassiveTicks);
            Number(velocity.LinearVelocity.Length()); Number(cockpitVelocity.LinearVelocity.Length());
            Number(_settleGate.ObservedSpeed); Number(velocity.AngularVelocity.Length());
            Number(_settleGate.ObservedAngularSpeed); Number(_settleGate.PassiveDrift); Number(_settleGate.PassiveRotation);
            _report.Line(_row.ToString());
            BeginCase();
        }

        void Brake(Vector3D linearVelocity)
        {
            _reference.SetAutoPilotEnabled(false);
            _reference.DampenersOverride = false;
            _battery.ChargeMode = ChargeMode.Auto; EnableReactors(true);
            Vector3D local = Vector3D.TransformNormal(linearVelocity, MatrixD.Transpose(_reference.WorldMatrix.GetOrientation()));
            for (int i = 0; i < 6; i++)
            {
                _thrusters[i].Enabled = true;
                _thrusters[i].ThrustOverridePercentage = (float)Math.Min(1, Math.Max(0, -Vector3D.Dot(local, Axis(i)) * _mass * 2 / _thrusters[i].MaxEffectiveThrust));
            }
            SetGyros(Vector3D.Zero, -1);
        }

        void BeginCase()
        {
            _trace.Observe("case_pre", _runTicks, 0, 0, _settleGate.StableTicks);
            ResetCommands(); _settling = false; _caseTick = 0; _caseSamples = 0;
            var test = _cases[_caseIndex];
            _report.Line("CASE|" + test.Name + "|" + test.Ticks);
            if (test.Kind == ProbeKind.Autopilot)
            {
                _reference.ClearWaypoints();
                _waypoint = _reference.GetPosition() + _reference.WorldMatrix.Forward * 30;
                _reference.AddWaypoint(_waypoint, "FC validation relative leg");
                _reference.FlightMode = FlightMode.OneWay;
                _reference.SpeedLimit = 3;
                _reference.SetAutoPilotEnabled(true);
            }
            ApplyCase(test);
            _trace.Observe("command", _runTicks, 0, 0, _settleGate.StableTicks);
            Sample(0);
            _echo("Measuring " + test.Name + "; phase trace and full samples are on FC Report.");
        }

        void TickCase(double dt)
        {
            var test = _cases[_caseIndex];
            _caseTick++;
            if (_caseTick <= 12 || _caseTick == 15 || _caseTick % 10 == 0 || _caseTick == test.Ticks
                || test.Kind == ProbeKind.Coast && _caseTick > 15 && _caseTick <= 27)
                _trace.Observe("flight", _runTicks, _caseTick, dt, 0);
            if (_caseTick % Math.Max(1, test.Ticks / 6) == 0 || _caseTick == test.Ticks) Sample(dt);
            if (_caseTick >= test.Ticks)
            {
                _report.Line("ENDCASE|" + test.Name + "|" + _caseSamples);
                ResetCommands(); _caseIndex++;
                if (_caseIndex == _cases.Length) { Finish("COMPLETE", _cases.Length.ToString()); return; }
                _settling = true; _settleTicks = 0; _passiveSettling = false;
                _settleGate.Reset(); _trace.BeginSettle(_cases[_caseIndex].Name);
            }
            else
            {
                ApplyCase(test);
                if (test.Kind == ProbeKind.Coast && _caseTick == 15)
                    _trace.Observe("command", _runTicks, _caseTick, 0, 0);
            }
        }

        void ApplyCase(ValidationCase test)
        {
            if (test.Kind == ProbeKind.Thrust) _thrusters[test.Axis].ThrustOverridePercentage = 1;
            if (test.Kind == ProbeKind.Coast) _thrusters[0].ThrustOverridePercentage = _caseTick < 15 ? 1 : 0;
            if (test.Kind == ProbeKind.Gyro || test.Kind == ProbeKind.GyroDecay)
            {
                int axis = test.Axis == 3 ? 0 : test.Axis;
                var rate = axis == 0 ? new Vector3D(0.4, 0, 0) : axis == 1 ? new Vector3D(0, 0.4, 0) : new Vector3D(0, 0, 0.4);
                if (test.Kind == ProbeKind.GyroDecay && _caseTick >= 60)
                { for (int i = 0; i < 2; i++) { _gyros[i].Enabled = true; _gyros[i].GyroOverride = false; } }
                else SetGyros(rate, test.Axis == 3 ? 1 : 0);
            }
            if (test.Kind == ProbeKind.Dampeners)
            {
                _thrusters[0].ThrustOverridePercentage = _caseTick < 60 ? 1 : 0;
                _reference.DampenersOverride = _caseTick >= 60;
            }
            if (test.Kind == ProbeKind.BatteryDischarge)
            { EnableReactors(false); _battery.ChargeMode = ChargeMode.Discharge; _thrusters[0].ThrustOverridePercentage = 0.2f; }
            if (test.Kind == ProbeKind.BatteryRecharge) _battery.ChargeMode = ChargeMode.Recharge;
            if (test.Kind == ProbeKind.LimitedPower)
            {
                EnableReactors(false); _battery.ChargeMode = ChargeMode.Discharge;
                for (int i = 0; i < 6; i++) _thrusters[i].ThrustOverridePercentage = 1;
                SetGyros(new Vector3D(0, 0.4, 0), -1);
            }
            if (test.Kind == ProbeKind.Connector)
            { if (_caseTick < 60) _connector.Connect(); else _connector.Disconnect(); }
        }

        void SetGyros(Vector3D bodyRate, int only)
        {
            var worldRate = Vector3D.TransformNormal(bodyRate, _reference.WorldMatrix);
            for (int i = 0; i < 2; i++)
            {
                var gyro = _gyros[i];
                gyro.Enabled = only < 0 || only == i;
                var local = Vector3D.TransformNormal(worldRate, MatrixD.Transpose(gyro.WorldMatrix.GetOrientation()));
                gyro.GyroPower = 1; gyro.GyroOverride = true;
                gyro.Pitch = -(float)local.X; gyro.Yaw = -(float)local.Y; gyro.Roll = -(float)local.Z;
            }
        }

        void Sample(double dt)
        {
            var velocity = _reference.GetShipVelocities();
            var p = _frame.Position(_reference.CenterOfMass);
            var v = _frame.Vector(velocity.LinearVelocity);
            var w = _frame.Vector(velocity.AngularVelocity);
            var q = _frame.Attitude(_reference.WorldMatrix);
            _row.Clear(); _row.Append("S|").Append(_cases[_caseIndex].Name).Append('|').Append(_caseTick);
            Number(_runTicks / 60.0); Number(dt);
            Number(p.X); Number(p.Y); Number(p.Z); Number(v.X); Number(v.Y); Number(v.Z);
            Number(w.X); Number(w.Y); Number(w.Z); Number(q.X); Number(q.Y); Number(q.Z); Number(q.W);
            for (int i = 0; i < 6; i++) Number(_thrusters[i].CurrentThrust);
            Number(_battery.CurrentInput); Number(_battery.CurrentOutput); Number(_battery.CurrentStoredPower - _initialBattery); Number(_reactor.CurrentOutput + _reactorB.CurrentOutput + _reactorC.CurrentOutput);
            Number(_reference.DampenersOverride ? 1 : 0); Number(_reference.IsAutoPilotEnabled ? 1 : 0); Number((int)_connector.Status);
            Number(_cases[_caseIndex].Kind == ProbeKind.Autopilot ? Vector3D.Distance(_reference.GetPosition(), _waypoint) : 0);
            _report.Line(_row.ToString()); _caseSamples++;
        }

        void Number(double value)
        {
            if (double.IsNaN(value) || double.IsInfinity(value)) throw new InvalidOperationException("NONFINITE_TELEMETRY");
            _row.Append('|').Append(ValidationReport.Number(value));
        }

        void ResetCommands()
        {
            for (int i = 0; i < 6; i++) if (_thrusters[i] != null) { _thrusters[i].ThrustOverridePercentage = 0; _thrusters[i].Enabled = true; }
            for (int i = 0; i < 2; i++) if (_gyros[i] != null)
            { _gyros[i].Pitch = 0; _gyros[i].Yaw = 0; _gyros[i].Roll = 0; _gyros[i].GyroOverride = false; _gyros[i].Enabled = true; }
            if (_reference != null) { _reference.SetAutoPilotEnabled(false); _reference.DampenersOverride = false; }
            if (_battery != null) _battery.ChargeMode = ChargeMode.Auto;
            EnableReactors(true);
        }

        void EnableReactors(bool enabled)
        {
            if (_reactor != null) _reactor.Enabled = enabled;
            if (_reactorB != null) _reactorB.Enabled = enabled;
            if (_reactorC != null) _reactorC.Enabled = enabled;
        }

        void Cleanup()
        {
            ResetCommands();
            if (_reference != null) _reference.DampenersOverride = true;
        }

        void Finish(string status, string reason)
        {
            _trace.Finish();
            Cleanup(); IsRunning = false; _runtime.UpdateFrequency = UpdateFrequency.None;
            _report.Line("END|" + status + "|" + reason + "|" + _report.Samples);
            _report.Publish(true);
            _echo("FC validation " + status + ": " + reason + ". Full report on FC Report LCD.");
        }

        void Dock()
        {
            // Explicit operator-assisted extension: never invent a docking success.
            if (_connector == null) { _echo("Missing FC Connector."); return; }
            var before = _connector.Status;
            if (before == MyShipConnectorStatus.Connectable) _connector.Connect();
            else if (before == MyShipConnectorStatus.Connected) _connector.Disconnect();
            _report.Line("MANUAL|dock|before=" + before + "|after=" + _connector.Status);
            _report.Publish(true);
            _echo("Dock status " + _connector.Status + ". Run dock again after a few ticks to record/disconnect.");
        }
    }
}
