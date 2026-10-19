namespace FleetCommand.Validation
{
    public enum ProbeKind { Observe, Thrust, Coast, Gyro, GyroDecay, Dampeners, BatteryDischarge, BatteryRecharge, LimitedPower, Autopilot, Connector, LowSpeed }

    /// <summary>Add cases here: all hosts consume this same ordered definition.</summary>
    public sealed class ValidationCase
    {
        public readonly string Name;
        public readonly ProbeKind Kind;
        public readonly int Axis;
        public readonly int Ticks;

        public ValidationCase(string name, ProbeKind kind, int axis, int ticks)
        {
            Name = name; Kind = kind; Axis = axis; Ticks = ticks;
        }

        public static ValidationCase[] CreateSuite()
        {
            return new[] {
                new ValidationCase("baseline", ProbeKind.Observe, 0, 120),
                new ValidationCase("thrust_px", ProbeKind.Thrust, 0, 90),
                new ValidationCase("coast_px", ProbeKind.Coast, 0, 60),
                new ValidationCase("thrust_nx", ProbeKind.Thrust, 1, 90),
                new ValidationCase("thrust_py", ProbeKind.Thrust, 2, 90),
                new ValidationCase("thrust_ny", ProbeKind.Thrust, 3, 90),
                new ValidationCase("thrust_pz", ProbeKind.Thrust, 4, 90),
                new ValidationCase("thrust_nz", ProbeKind.Thrust, 5, 90),
                new ValidationCase("gyro_pitch", ProbeKind.Gyro, 0, 180),
                new ValidationCase("gyro_yaw", ProbeKind.Gyro, 1, 180),
                new ValidationCase("gyro_roll", ProbeKind.Gyro, 2, 180),
                new ValidationCase("gyro_rotated_mount", ProbeKind.Gyro, 3, 180),
                new ValidationCase("gyro_decay", ProbeKind.GyroDecay, 0, 180),
                new ValidationCase("dampener_braking", ProbeKind.Dampeners, 0, 300),
                new ValidationCase("battery_discharge", ProbeKind.BatteryDischarge, 0, 300),
                new ValidationCase("battery_recharge", ProbeKind.BatteryRecharge, 0, 300),
                new ValidationCase("limited_power", ProbeKind.LimitedPower, 0, 180),
                new ValidationCase("waypoint_forward", ProbeKind.Autopilot, 0, 600),
                new ValidationCase("connector_unpaired", ProbeKind.Connector, 0, 120)
            };
        }

        public static ValidationCase[] CreateLowSpeedDiagnostic()
        {
            // Axis selects the intervention for this diagnostic kind.
            return new[] {
                new ValidationCase("low_hold", ProbeKind.LowSpeed, 0, 300),
                new ValidationCase("low_wake", ProbeKind.LowSpeed, 1, 300),
                new ValidationCase("low_refresh", ProbeKind.LowSpeed, 2, 300),
                new ValidationCase("low_moving", ProbeKind.LowSpeed, 3, 300),
                new ValidationCase("low_gyro_zero", ProbeKind.LowSpeed, 4, 300),
                new ValidationCase("low_gyro_spin", ProbeKind.LowSpeed, 5, 300)
            };
        }
    }
}
