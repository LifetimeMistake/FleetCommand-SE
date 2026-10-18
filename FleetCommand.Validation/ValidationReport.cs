using System.Text;
using Sandbox.ModAPI.Ingame;
using VRage.Game.GUI.TextPanel;

namespace FleetCommand.Validation
{
    /// <summary>The LCD is the complete canonical artifact, not just a status display.</summary>
    public sealed class ValidationReport
    {
        public const int Limit = 94000;
        readonly StringBuilder _data = new StringBuilder();
        readonly IMyTextPanel _lcd;
        int _published;
        public bool Overflow { get; private set; }
        public int Samples { get; private set; }
        public string Text { get { return _data.ToString(); } }

        public ValidationReport(IMyTextPanel lcd, string previous)
        {
            _lcd = lcd;
            if (!string.IsNullOrEmpty(previous) && previous.Length < Limit) _data.Append(previous);
            if (_lcd != null) { _lcd.ContentType = ContentType.TEXT_AND_IMAGE; _lcd.FontSize = 0.5f; }
        }

        public void Reset()
        {
            _data.Clear(); _published = 0; Overflow = false; Samples = 0;
            if (_lcd != null) _lcd.WriteText("");
        }

        public bool Line(string line)
        {
            if (Overflow) return false;
            if (_data.Length + line.Length + 1 > Limit - 256)
            {
                _data.Append("END|ERROR|REPORT_OVERFLOW\n"); Overflow = true; return false;
            }
            _data.Append(line).Append('\n');
            if (line.StartsWith("S|")) Samples++;
            return true;
        }

        public bool Publish(bool replace)
        {
            if (_lcd == null) return false;
            if (replace) _published = 0;
            if (_lcd.WriteText(_data.ToString(_published, _data.Length - _published), _published != 0))
            { _published = _data.Length; return true; }
            return false;
        }

        public static string Number(double value)
        {
            // The PB wrapper does not import System.Globalization. Fixed-point
            // formatting has no grouping; normalize the decimal comma explicitly.
            return value.ToString("0.000000").Replace(',', '.');
        }
    }
}
