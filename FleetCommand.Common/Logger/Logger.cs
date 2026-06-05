using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace FleetCommand.Common.Logger
{
    public class Logger
    {
        private IMyTextPanel _lcd;
        private Action<string> _echo;
        private RingBuffer<LogEntry> _buffer;

        public Logger(int logLimit, IMyTextPanel lcd = null, Action<string> echo = null)
        {
            _lcd = lcd;
            _echo = echo;
            _buffer = new RingBuffer<LogEntry>(logLimit);
        }

        public void Debug(string message, string type = null)
        {
            Log(message, LogLevel.DEBUG, type);
        }
        
        public void Info(string message, string type = null)
        {
            Log(message, LogLevel.INFO, type);
        }

        public void Warning(string message, string type = null)
        {
            Log(message, LogLevel.WARN, type);
        }

        public void Error(string message, string type = null)
        {
            Log(message, LogLevel.ERROR, type);
        }

        public void Clear()
        {
            _buffer.Clear();
            PrintLogs();
        }

        private void PrintLogs()
        {
            string logs = string.Join("\n", _buffer);
            if (_lcd != null)
                _lcd.WriteText(logs);
            if (_echo != null)
                _echo(logs);
        }

        private void Log(string message, LogLevel level, string type = null)
        {
            LogEntry entry = new LogEntry 
            { 
                Timestamp = TimeSource.Timestamp, 
                Level = level, 
                Type = type, 
                Message = message 
            };
            _buffer.Push(entry);
            PrintLogs();
        }
    }
}
