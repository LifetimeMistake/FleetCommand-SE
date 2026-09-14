using System;
using System.Text;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRage.Game.GUI.TextPanel;
using VRageMath;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// Text panel / LCD, implementing the real <see cref="IMyTextPanel"/>
    /// (from Sandbox.ModAPI.Ingame; , ) — which extends
    /// <see cref="IMyTextSurface"/> — for logger-compatible text writes and
    /// reads. The implemented contract is the text model (<see cref="WriteText"/>,
    /// <see cref="GetText"/>, <see cref="ReadText"/>, public title) plus
    /// the stored surface attributes (font, colors, alignment, content
    /// type, …). The sprite / drawing surface (images, sprites,
    /// <see cref="DrawFrame"/>, pixel measurement) models rendering the
    /// simulator does not provide, so those members throw a naming
    /// <see cref="NotSupportedException"/>.
    /// </summary>
    public sealed class SimTextPanel : SimFunctionalBlock, IMyTextPanel
    {
        /// <summary>Cap on public text length (per the interface docs: 100000 symbols).</summary>
        public const int MaxTextLength = 100000;

        private readonly StringBuilder _text = new StringBuilder();
        private string _title = string.Empty;

        public SimTextPanel(long entityId, string name, SimCubeGrid grid, TemplateBlock template)
            : base(entityId, name, grid, template)
        {
        }

        // — the text model —

        /// <inheritdoc />
        public bool WriteText(string value, bool append = false)
        {
            ApplyText(value, append);
            return true; // no popup viewer is open
        }

        /// <inheritdoc />
        public bool WriteText(StringBuilder value, bool append = false)
        {
            ApplyText(value == null ? string.Empty : value.ToString(), append);
            return true;
        }

        /// <inheritdoc />
        public string GetText()
        {
            return _text.ToString();
        }

        /// <inheritdoc />
        public void ReadText(StringBuilder buffer, bool append = false)
        {
            if (buffer == null)
            {
                throw new ArgumentNullException(nameof(buffer));
            }
            if (!append)
            {
                buffer.Clear();
            }
            buffer.Append(_text);
        }

        /// <inheritdoc />
        public bool WritePublicTitle(string value, bool append = false)
        {
            string content = value ?? string.Empty;
            _title = append ? _title + content : content;
            return true;
        }

        /// <inheritdoc />
        public string GetPublicTitle()
        {
            return _title;
        }

        private void ApplyText(string value, bool append)
        {
            string content = value ?? string.Empty;
            if (append)
            {
                _text.Append(content);
            }
            else
            {
                _text.Clear();
                _text.Append(content);
            }
            if (_text.Length > MaxTextLength)
            {
                _text.Length = MaxTextLength;
            }
        }

        // — stored surface attributes —

        /// <summary>Surface identifier name; for a single-surface block this is the block's custom name.</summary>
        public new string Name => CustomName;

        /// <inheritdoc />
        public override string DisplayName => CustomName;

        /// <inheritdoc />
        public float FontSize { get; set; } = 1f;

        /// <inheritdoc />
        public Color FontColor { get; set; } = Color.White;

        /// <inheritdoc />
        public Color BackgroundColor { get; set; } = Color.Black;

        /// <inheritdoc />
        public byte BackgroundAlpha { get; set; }

        /// <inheritdoc />
        public float ChangeInterval { get; set; }

        /// <inheritdoc />
        public string Font { get; set; } = "Debug";

        /// <inheritdoc />
        public TextAlignment Alignment { get; set; }

        /// <inheritdoc />
        public string Script { get; set; }

        /// <inheritdoc />
        public ContentType ContentType { get; set; } = ContentType.TEXT_AND_IMAGE;

        /// <inheritdoc />
        public bool PreserveAspectRatio { get; set; }

        /// <inheritdoc />
        public float TextPadding { get; set; }

        /// <inheritdoc />
        public Color ScriptBackgroundColor { get; set; } = Color.Black;

        /// <inheritdoc />
        public Color ScriptForegroundColor { get; set; } = Color.White;

        // — IMyTextSurface render-only members (no rendering in the simulator) —

        /// <inheritdoc />
        public string CurrentlyShownImage => ThrowNotSupported<string>("IMyTextSurface.CurrentlyShownImage");

        /// <inheritdoc />
        public Vector2 SurfaceSize => ThrowNotSupported<Vector2>("IMyTextSurface.SurfaceSize");

        /// <inheritdoc />
        public Vector2 TextureSize => ThrowNotSupported<Vector2>("IMyTextSurface.TextureSize");

        /// <inheritdoc />
        public void AddImageToSelection(string id, bool checkExistence = false) => ThrowNotSupportedVoid("IMyTextSurface.AddImageToSelection()");

        /// <inheritdoc />
        public void AddImagesToSelection(System.Collections.Generic.List<string> ids, bool checkExistence = false) => ThrowNotSupportedVoid("IMyTextSurface.AddImagesToSelection()");

        /// <inheritdoc />
        public void RemoveImageFromSelection(string id, bool removeDuplicates = false) => ThrowNotSupportedVoid("IMyTextSurface.RemoveImageFromSelection()");

        /// <inheritdoc />
        public void RemoveImagesFromSelection(System.Collections.Generic.List<string> ids, bool removeDuplicates = false) => ThrowNotSupportedVoid("IMyTextSurface.RemoveImagesFromSelection()");

        /// <inheritdoc />
        public void ClearImagesFromSelection() => ThrowNotSupportedVoid("IMyTextSurface.ClearImagesFromSelection()");

        /// <inheritdoc />
        public void GetSelectedImages(System.Collections.Generic.List<string> output) => ThrowNotSupportedVoid("IMyTextSurface.GetSelectedImages()");

        /// <inheritdoc />
        public void GetFonts(System.Collections.Generic.List<string> fonts) => ThrowNotSupportedVoid("IMyTextSurface.GetFonts()");

        /// <inheritdoc />
        public void GetSprites(System.Collections.Generic.List<string> sprites) => ThrowNotSupportedVoid("IMyTextSurface.GetSprites()");

        /// <inheritdoc />
        public void GetScripts(System.Collections.Generic.List<string> scripts) => ThrowNotSupportedVoid("IMyTextSurface.GetScripts()");

        /// <inheritdoc />
        public MySpriteDrawFrame DrawFrame() => ThrowNotSupported<MySpriteDrawFrame>("IMyTextSurface.DrawFrame()");

        /// <inheritdoc />
        public Vector2 MeasureStringInPixels(StringBuilder text, string font, float scale) => ThrowNotSupported<Vector2>("IMyTextSurface.MeasureStringInPixels()");
    }
}
