using System;
using System.Collections.Generic;
using System.Text;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Validation
{
    /// <summary>Bounded transition evidence, exported on the same LCD as canonical samples.</summary>
    public sealed class ValidationTrace
    {
        public const int Budget = 48000;
        public const string Columns = "DCOLUMNS|case|phase|run_tick|phase_tick|dt|com_px|com_py|com_pz|rc_px|rc_py|rc_pz|pb_px|pb_py|pb_pz|qx|qy|qz|qw|rc_vx|rc_vy|rc_vz|cockpit_vx|cockpit_vy|cockpit_vz|omega|position_speed|stable_ticks|dampeners|autopilot|pilot|remote_pilot|thrust_enabled_mask|thrust_working_mask|gyro_enabled_mask|gyro_override_mask|override_px|override_nx|override_py|override_ny|override_pz|override_nz|thrust_px|thrust_nx|thrust_py|thrust_ny|thrust_pz|thrust_nz";
        readonly ValidationReport _report;
        readonly IMyRemoteControl _reference;
        readonly IMyCockpit _cockpit;
        readonly IMyProgrammableBlock _me;
        readonly IMyThrust[] _thrusters;
        readonly IMyGyro[] _gyros;
        readonly Queue<string> _brakeTail = new Queue<string>();
        readonly StringBuilder _row = new StringBuilder();
        InitialFrame _frame;
        string _case;
        bool _selected, _finished, _havePosition;
        Vector3D _previousPosition;
        int _previousTick, _characters, _rows, _dropped;

        public ValidationTrace(ValidationReport report, IMyRemoteControl reference, IMyCockpit cockpit,
            IMyProgrammableBlock me, IMyThrust[] thrusters, IMyGyro[] gyros)
        { _report = report; _reference = reference; _cockpit = cockpit; _me = me; _thrusters = thrusters; _gyros = gyros; }

        public void Reset(InitialFrame frame)
        {
            _frame = frame; _finished = false; _havePosition = false;
            _characters = 0; _rows = 0; _dropped = 0; _brakeTail.Clear();
        }

        public void BeginSettle(string name)
        {
            _case = name; _selected = name == "thrust_px" || name == "coast_px" || name == "thrust_py";
            _havePosition = false; _brakeTail.Clear();
        }

        public void Observe(string phase, int runTick, int phaseTick, double dt, int stable)
        {
            if (!_selected) return;
            string line = Capture(phase, runTick, phaseTick, dt, stable);
            if (phase == "brake")
            {
                if (_brakeTail.Count == 8) _brakeTail.Dequeue();
                _brakeTail.Enqueue(line);
            }
            else Write(line);
        }

        public void FlushBrakeTail()
        {
            while (_brakeTail.Count > 0) Write(_brakeTail.Dequeue());
        }

        string Capture(string phase, int runTick, int phaseTick, double dt, int stable)
        {
            var velocity = _reference.GetShipVelocities();
            var cockpitVelocity = _cockpit.GetShipVelocities();
            Vector3D com = _reference.CenterOfMass;
            double speed = _havePosition && runTick > _previousTick
                ? Vector3D.Distance(com, _previousPosition) * 60 / (runTick - _previousTick) : 0;
            _previousPosition = com; _previousTick = runTick; _havePosition = true;
            _row.Clear(); _row.Append("D|").Append(_case).Append('|').Append(phase).Append('|').Append(runTick).Append('|').Append(phaseTick);
            Number(dt); Vector(_frame.Position(com)); Vector(_frame.Position(_reference.GetPosition()));
            Vector(_frame.Position(_me.GetPosition()));
            var q = _frame.Attitude(_reference.WorldMatrix);
            Number(q.X); Number(q.Y); Number(q.Z); Number(q.W);
            Vector(_frame.Vector(velocity.LinearVelocity));
            Vector(_frame.Vector(cockpitVelocity.LinearVelocity));
            Number(velocity.AngularVelocity.Length()); Number(speed); Number(stable);
            Number(_reference.DampenersOverride ? 1 : 0); Number(_reference.IsAutoPilotEnabled ? 1 : 0);
            Number(_cockpit.IsUnderControl ? 1 : 0); Number(_reference.IsUnderControl ? 1 : 0);
            int thrustEnabled = 0, thrustWorking = 0, gyroEnabled = 0, gyroOverride = 0;
            for (int i = 0; i < 6; i++)
            { if (_thrusters[i].Enabled) thrustEnabled |= 1 << i; if (_thrusters[i].IsWorking) thrustWorking |= 1 << i; }
            for (int i = 0; i < 2; i++)
            { if (_gyros[i].Enabled) gyroEnabled |= 1 << i; if (_gyros[i].GyroOverride) gyroOverride |= 1 << i; }
            Number(thrustEnabled, true); Number(thrustWorking, true); Number(gyroEnabled, true); Number(gyroOverride, true);
            for (int i = 0; i < 6; i++) Number(_thrusters[i].ThrustOverridePercentage, true);
            for (int i = 0; i < 6; i++) Number(_thrusters[i].CurrentThrust, true);
            return _row.ToString();
        }

        void Vector(Vector3D value) { Number(value.X); Number(value.Y); Number(value.Z); }
        void Number(double value, bool compact = false)
        {
            if (double.IsNaN(value) || double.IsInfinity(value)) throw new InvalidOperationException("NONFINITE_DIAGNOSTIC");
            _row.Append('|').Append(compact ? value.ToString("0.000").Replace(',', '.') : ValidationReport.Number(value));
        }
        void Write(string line)
        {
            if (_characters + line.Length + 1 > Budget) { _dropped++; return; }
            _characters += line.Length + 1; _rows++; _report.Line(line);
        }
        public void Finish()
        {
            if (_frame == null || _finished) return;
            _finished = true; FlushBrakeTail();
            _report.Line("TRACE|rows|" + _rows + "|dropped|" + _dropped + "|characters|" + _characters);
        }
    }
}
