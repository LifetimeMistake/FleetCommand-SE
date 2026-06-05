using FleetCommand.Common.Logger;
using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Common.Logger
{
    public struct LogEntry
    {
        public TimeSpan Timestamp { get; set; }
        public LogLevel Level { get; set; }
        public string Type { get; set; }
        public string Message { get; set; }

        public override string ToString()
        {
            return $"({Timestamp.Hours:00}:{Timestamp.Minutes:00}:{Timestamp.Seconds:00}) " +
                $"<{Level}>" +
                (Type != null ? $" {Type}" : "") + ": " +
                $"{Message}";
        }
    }
}
