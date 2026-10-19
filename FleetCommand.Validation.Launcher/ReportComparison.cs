using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using VRageMath;

namespace FleetCommand.Validation.Launcher
{
    public sealed class ComparisonResult
    {
        public List<string> Differences { get; } = new List<string>();
        public bool Success => Differences.Count == 0;
        public override string ToString() => (Success ? "PASS" : "FAIL")
            + " (provisional tolerances; measured game equivalence requires a real game report)\n"
            + string.Join("\n", Differences);
    }

    public static class ReportComparison
    {
        sealed class Parsed
        {
            public readonly Dictionary<string, double[]> Samples = new Dictionary<string, double[]>();
            public readonly Dictionary<string, double> Meta = new Dictionary<string, double>();
            public readonly List<string> Contract = new List<string>();
            public readonly Dictionary<string, int> Settles = new Dictionary<string, int>();
            public MatrixD? InitialOrientation;
        }

        public static ComparisonResult Compare(string expected, string observed)
        {
            var result = new ComparisonResult();
            Parsed left, right;
            try { left = Parse(expected); } catch (Exception e) { result.Differences.Add("Invalid first report: " + e.Message); return result; }
            try { right = Parse(observed); } catch (Exception e) { result.Differences.Add("Invalid second report: " + e.Message); return result; }
            if (left.InitialOrientation.HasValue != right.InitialOrientation.HasValue)
                result.Differences.Add("Missing initial world orientation in one report; rerun the updated script.");
            else if (left.InitialOrientation.HasValue
                && (Vector3D.Distance(left.InitialOrientation.Value.Right, right.InitialOrientation.Value.Right) > 0.000005
                    || Vector3D.Distance(left.InitialOrientation.Value.Up, right.InitialOrientation.Value.Up) > 0.000005))
                result.Differences.Add("Incompatible initial world orientation: replay with --frame-from game-report.txt.");
            if (!left.Contract.SequenceEqual(right.Contract)) result.Differences.Add("Schema, cases, block capabilities or coverage differ.");
            foreach (var pair in left.Meta)
            {
                double actual;
                if (pair.Key.StartsWith("initial_q")) continue; // compare the complete orientation above
                if (!right.Meta.TryGetValue(pair.Key, out actual)) { result.Differences.Add("Missing metadata: " + pair.Key); continue; }
                if (pair.Key == "battery_start_mwh") continue; // compare charge changes; preflight bounds capacity dependence
                double tolerance = pair.Key == "mass_kg" ? 0.05 : 0;
                if (Math.Abs(pair.Value - actual) > tolerance) result.Differences.Add((pair.Key == "terminal_blocks" ? "API discovery differs: " : "Incompatible initial conditions: ") + pair.Key + " " + pair.Value + " vs " + actual);
            }
            if (result.Differences.Any(d => !d.StartsWith("API discovery differs:"))) return result;
            foreach (var settle in left.Settles)
                if (Math.Abs(settle.Value - right.Settles[settle.Key]) > 1)
                    result.Differences.Add(settle.Key + ": settling ticks " + settle.Value + " vs " + right.Settles[settle.Key]);
            string[] columns = ValidationRunner.Columns.Split('|').Skip(3).ToArray();
            foreach (var pair in left.Samples)
            {
                double[] values = right.Samples[pair.Key];
                for (int i = 0; i < values.Length; i++)
                {
                    if (i >= 11 && i <= 14) continue; // compare attitude as a rotation below
                    var tolerance = Tolerance(columns[i]);
                    double bound = tolerance.Item1 + tolerance.Item2 * Math.Max(Math.Abs(pair.Value[i]), Math.Abs(values[i]));
                    if (Math.Abs(pair.Value[i] - values[i]) > bound)
                        result.Differences.Add(pair.Key + " " + columns[i] + ": " + F(pair.Value[i]) + " vs " + F(values[i]) + " (limit " + F(bound) + ")");
                }
                double dot = 0, normA = 0, normB = 0;
                for (int i = 11; i <= 14; i++) { dot += pair.Value[i] * values[i]; normA += pair.Value[i] * pair.Value[i]; normB += values[i] * values[i]; }
                double angle = 2 * Math.Acos(Math.Min(1, Math.Abs(dot) / Math.Sqrt(normA * normB)));
                if (angle > 0.01) result.Differences.Add(pair.Key + " attitude: " + F(angle) + " rad (limit 0.01)");
            }
            return result;
        }

        static string F(double number) => number.ToString("G8", CultureInfo.InvariantCulture);

        static Tuple<double, double> Tolerance(string column)
        {
            if (column == "seconds") return Tuple.Create(0.05, 0.0);
            if (column == "dt") return Tuple.Create(0.001, 0.0);
            if (column.StartsWith("thrust_")) return Tuple.Create(0.5, 0.01);
            if (column == "battery_delta_mwh") return Tuple.Create(0.00001, 0.001);
            if (column == "battery_in" || column == "battery_out" || column == "reactor_mw") return Tuple.Create(0.001, 0.01);
            if (column == "dampeners" || column == "autopilot" || column == "connector") return Tuple.Create(0.0, 0.0);
            if (column.StartsWith("w")) return Tuple.Create(column == "waypoint_m" ? 0.1 : 0.005, 0.01);
            if (column.StartsWith("v")) return Tuple.Create(0.03, 0.01);
            return Tuple.Create(0.05, 0.01);
        }

        static double Number(string value)
        {
            double result;
            if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out result)
                || double.IsNaN(result) || double.IsInfinity(result)) throw new FormatException("Invalid finite number: " + value);
            return result;
        }

        static Parsed Parse(string report)
        {
            if (string.IsNullOrEmpty(report) || report.Length >= 100000) throw new FormatException("Empty or oversized report.");
            string[] lines = report.Replace("\r", "").TrimEnd('\n').Split('\n');
            if (lines[0] != ValidationRunner.Header && lines[0] != "FCVALIDATION|1|craft-v1|suite-v1"
                && lines[0] != "FCVALIDATION|1|craft-v1|suite-v2" && lines[0] != "FCVALIDATION|1|craft-v1|suite-v3"
                && lines[0] != "FCVALIDATION|1|craft-v1|suite-v4")
                throw new FormatException("Unknown report revision.");
            var parsed = new Parsed();
            parsed.Contract.Add(lines[0]);
            var suite = ValidationCase.CreateSuite();
            int completed = 0, caseSamples = 0, previousTick = -1;
            bool open = false, ended = false, columns = false, lcdCheck = false;
            bool startupInfo = false;
            bool waypointDetailed = lines[0] == ValidationRunner.Header || lines[0] == "FCVALIDATION|1|craft-v1|suite-v4";
            bool detailed = waypointDetailed || lines[0] == "FCVALIDATION|1|craft-v1|suite-v3";
            bool waypointColumns = false;
            var waypointTicks = new HashSet<int>();
            bool gateColumns = false, gatePolicy = false, traceColumns = false, traceFooter = false;
            int traceRows = 0, traceCharacters = 0;
            var gates = new HashSet<string>();
            var traceCases = new HashSet<string>();
            double previousTime = -1;
            var coverage = new HashSet<string>();
            foreach (string line in lines.Skip(1))
            {
                var cells = line.Split('|');
                if (ended)
                {
                    if (cells[0] != "MANUAL") throw new FormatException("Data after completion marker.");
                    continue;
                }
                switch (cells[0])
                {
                    case "INFO":
                        if (startupInfo || columns || cells.Length != 3 || cells[1] != "startup_zero_elapsed_callbacks"
                            || !int.TryParse(cells[2], out int skipped) || skipped < 1 || skipped > 10)
                            throw new FormatException("Invalid startup timing diagnostic.");
                        startupInfo = true; break;
                    case "META":
                        if (cells.Length != 3) throw new FormatException("Malformed metadata.");
                        parsed.Meta.Add(cells[1], Number(cells[2])); break;
                    case "COVERAGE":
                        if (cells.Length != 3 || !coverage.Add(cells[1])) throw new FormatException("Malformed coverage.");
                        parsed.Contract.Add(line); break;
                    case "BLOCK":
                        if (cells.Length != 4 || Number(cells[3]) <= 0) throw new FormatException("Malformed block capability.");
                        parsed.Contract.Add(line); break;
                    case "CHECK":
                        if (line != "CHECK|lcd_append_read|PASS" || lcdCheck) throw new FormatException("LCD probe missing or failed.");
                        lcdCheck = true; parsed.Contract.Add(line); break;
                    case "COLUMNS":
                        if (line != ValidationRunner.Columns || columns) throw new FormatException("Unexpected columns.");
                        columns = true; break;
                    case "GATECOLUMNS":
                        if (!detailed || !columns || gateColumns || line != ValidationRunner.GateColumns)
                            throw new FormatException("Invalid settling columns.");
                        gateColumns = true; parsed.Contract.Add(line); break;
                    case "GATEPOLICY":
                        if (!detailed || !gateColumns || gatePolicy || line != ValidationRunner.GatePolicy)
                            throw new FormatException("Invalid settling policy.");
                        gatePolicy = true; parsed.Contract.Add(line); break;
                    case "DCOLUMNS":
                        if (!detailed || !gatePolicy || traceColumns || line != ValidationTrace.Columns)
                            throw new FormatException("Invalid diagnostic columns.");
                        traceColumns = true; parsed.Contract.Add(line); break;
                    case "WPCOLUMNS":
                        if (!waypointDetailed || !traceColumns || waypointColumns || open || line != ValidationRunner.WaypointColumns)
                            throw new FormatException("Invalid waypoint columns.");
                        waypointColumns = true; parsed.Contract.Add(line); break;
                    case "W":
                        if (!waypointDetailed || !waypointColumns || !open || completed >= suite.Length
                            || suite[completed].Kind != ProbeKind.Autopilot || cells.Length != ValidationRunner.WaypointColumns.Split('|').Length
                            || !int.TryParse(cells[1], out int waypointTick) || waypointTick < 0 || waypointTick > 600
                            || waypointTick > 60 && waypointTick % 20 != 0 || !waypointTicks.Add(waypointTick)
                            || waypointTick != (waypointTicks.Count <= 61 ? waypointTicks.Count - 1 : 60 + (waypointTicks.Count - 61) * 20))
                            throw new FormatException("Invalid waypoint sample.");
                        var waypointValues = cells.Skip(2).Select(Number).ToArray();
                        if (waypointValues[0] < 0 || waypointValues.Skip(4).Any(v => v < 0))
                            throw new FormatException("Invalid waypoint telemetry.");
                        break;
                    case "D":
                        if (!detailed || !traceColumns || traceFooter || completed >= suite.Length || cells[1] != suite[completed].Name
                            || !new[] { "thrust_px", "coast_px", "thrust_py" }.Contains(cells[1])
                            || cells.Length != ValidationTrace.Columns.Split('|').Length
                            || !new[] { "brake", "passive", "passive_failed", "case_pre", "command", "flight" }.Contains(cells[2]))
                            throw new FormatException("Invalid diagnostic row.");
                        var diagnostic = cells.Skip(3).Select(Number).ToArray();
                        if (diagnostic[0] < 0 || diagnostic[1] < 0 || diagnostic[2] < 0 || diagnostic[2] > 0.025)
                            throw new FormatException("Invalid diagnostic timing.");
                        traceRows++; traceCharacters += line.Length + 1;
                        if (cells[2] == "flight") traceCases.Add(cells[1]);
                        break;
                    case "GATE":
                        if (!detailed || !gatePolicy || open || completed >= suite.Length || cells[1] != suite[completed].Name
                            || cells.Length != ValidationRunner.GateColumns.Split('|').Length || !gates.Add(cells[1]))
                            throw new FormatException("Invalid settling evidence.");
                        var evidence = cells.Skip(2).Select(Number).ToArray();
                        if (!parsed.Settles.ContainsKey(cells[1]) || evidence[0] != parsed.Settles[cells[1]]
                            || evidence[1] != SettlingGate.BrakeFrames || evidence[2] != SettlingGate.CoastFrames
                            || evidence.Skip(3).Any(v => v < 0 || v > 0.010001))
                            throw new FormatException("Settling gate did not satisfy its policy.");
                        parsed.Contract.Add("GATE|" + cells[1]); break;
                    case "TRACE":
                        if (!detailed || traceFooter || cells.Length != 7 || cells[1] != "rows" || cells[3] != "dropped"
                            || cells[5] != "characters" || Number(cells[2]) != traceRows || Number(cells[4]) != 0
                            || Number(cells[6]) != traceCharacters || traceCharacters > (waypointDetailed ? ValidationTrace.Budget : 48000))
                            throw new FormatException("Missing or truncated diagnostic trace.");
                        traceFooter = true; break;
                    case "SETTLE":
                        if (cells.Length != 3 || completed >= suite.Length || cells[1] != suite[completed].Name) throw new FormatException("Invalid settling record.");
                        int ticks = int.Parse(cells[2], CultureInfo.InvariantCulture);
                        if (ticks < (detailed ? SettlingGate.BrakeFrames + SettlingGate.CoastFrames : 15) || ticks > 900)
                            throw new FormatException("Invalid settling duration.");
                        parsed.Settles.Add(cells[1], ticks); break;
                    case "CASE":
                        if (open || completed >= suite.Length || cells.Length != 3 || cells[1] != suite[completed].Name
                            || Number(cells[2]) != suite[completed].Ticks || !parsed.Settles.ContainsKey(cells[1])) throw new FormatException("Missing, reordered or invalid case.");
                        open = true; caseSamples = 0; previousTick = -1; parsed.Contract.Add(line); break;
                    case "S":
                        if (!open || !columns || cells.Length != ValidationRunner.Columns.Split('|').Length || cells[1] != suite[completed].Name)
                            throw new FormatException("Malformed telemetry row.");
                        int tick = int.Parse(cells[2], CultureInfo.InvariantCulture);
                        if (tick != caseSamples * (suite[completed].Ticks / 6) || tick <= previousTick || tick > suite[completed].Ticks)
                            throw new FormatException("Missing or duplicate samples.");
                        double[] sample = cells.Skip(3).Select(Number).ToArray();
                        if (sample[0] < previousTime || sample[1] < 0 || sample[1] > 0.025 || tick > 0 && sample[1] < 0.010)
                            throw new FormatException("Invalid sample timing.");
                        double norm = sample.Skip(11).Take(4).Sum(x => x * x);
                        if (Math.Abs(norm - 1) > 0.001) throw new FormatException("Invalid attitude quaternion.");
                        parsed.Samples.Add(cells[1] + "|" + cells[2], sample);
                        previousTick = tick; previousTime = sample[0]; caseSamples++; break;
                    case "ENDCASE":
                        if (!open || cells.Length != 3 || cells[1] != suite[completed].Name || Number(cells[2]) != caseSamples
                            || previousTick != suite[completed].Ticks || caseSamples != 7) throw new FormatException("Incomplete case.");
                        open = false; completed++; parsed.Contract.Add(line); break;
                    case "END":
                        if (open || cells.Length != 4 || cells[1] != "COMPLETE" || Number(cells[2]) != suite.Length
                            || Number(cells[3]) != parsed.Samples.Count || completed != suite.Length) throw new FormatException("Incomplete or failed run.");
                        ended = true; break;
                    default: throw new FormatException("Unknown record: " + cells[0]);
                }
            }
            var metadataNames = new List<string> { "battery_start_mwh", "grid_size_m", "mass_kg", "terminal_blocks" };
            if (lines[0] == ValidationRunner.Header || parsed.Meta.Keys.Any(k => k.StartsWith("initial_q")))
            {
                metadataNames.AddRange(new[] { "initial_qx", "initial_qy", "initial_qz", "initial_qw" });
                parsed.InitialOrientation = ReportInitialOrientation.Read(report);
            }
            if (!ended || !lcdCheck || !coverage.IsSupersetOf(new[] { "automated", "manual", "uncovered", "limitations" })
                || !parsed.Meta.Keys.OrderBy(x => x).SequenceEqual(metadataNames.OrderBy(x => x)))
                throw new FormatException("Missing completion, metadata, LCD probe or coverage.");
            if (parsed.Contract.Count(l => l.StartsWith("BLOCK|")) != 6) throw new FormatException("Missing thruster capabilities.");
            if (detailed && (!gateColumns || !gatePolicy || !traceColumns || !traceFooter || gates.Count != suite.Length
                || !traceCases.SetEquals(new[] { "thrust_px", "coast_px", "thrust_py" })))
                throw new FormatException("Missing settling or transition diagnostics.");
            if (waypointDetailed && (!waypointColumns || waypointTicks.Count != 88))
                throw new FormatException("Missing waypoint diagnostics.");
            if (parsed.Meta["battery_start_mwh"] < 0.02 || parsed.Meta["battery_start_mwh"] > 0.04 || parsed.Meta["mass_kg"] <= 0)
                throw new FormatException("Invalid preflight conditions.");
            return parsed;
        }
    }
}
