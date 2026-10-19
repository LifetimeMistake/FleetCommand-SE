using System;
using System.Globalization;
using VRageMath;

namespace FleetCommand.Validation.Launcher
{
    /// <summary>World orientation of the reference controller, retained for native solver replay.</summary>
    public static class ReportInitialOrientation
    {
        public static MatrixD Read(string report)
        {
            var values = new double[4];
            var found = new bool[4];
            string[] names = { "initial_qx", "initial_qy", "initial_qz", "initial_qw" };
            foreach (string line in report.Replace("\r", "").Split('\n'))
            {
                var cells = line.Split('|');
                if (cells.Length < 2 || cells[0] != "META") continue;
                int index = Array.IndexOf(names, cells[1]);
                if (index < 0) continue;
                double value;
                if (found[index] || cells.Length != 3
                    || !double.TryParse(cells[2], NumberStyles.Float, CultureInfo.InvariantCulture, out value)
                    || double.IsNaN(value) || double.IsInfinity(value))
                    throw new FormatException("Invalid initial orientation: " + cells[1]);
                found[index] = true;
                values[index] = value;
            }
            for (int i = 0; i < 4; i++)
                if (!found[i]) throw new FormatException("Report is missing initial orientation. Run the updated script first.");
            var q = new QuaternionD(values[0], values[1], values[2], values[3]);
            if (Math.Abs(q.LengthSquared() - 1) > 0.00001)
                throw new FormatException("Initial orientation is not a unit quaternion.");
            q.Normalize();
            return MatrixD.CreateFromQuaternion(q);
        }
    }
}
