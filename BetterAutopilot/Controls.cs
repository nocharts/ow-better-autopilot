using System;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;

namespace BetterAutopilot
{
    /// <summary>
    /// Cockpit hotkeys: cycle the autopilot thruster mode (game default / 2 / 3), and toggle
    /// auto-align during autopilot. Both changes are saved to the mod config.
    /// </summary>
    public static class Controls
    {
        public static Key CycleKey = Key.J;
        public static Key ToggleAlignKey = Key.G;
        // D-pad left/right are free in the cockpit (only the scout camera uses them, and we skip
        // those while a tool is out). Up is autopilot, down is free look.
        public static GamepadButton? CycleButton = GamepadButton.DpadRight;
        public static GamepadButton? ToggleAlignButton = GamepadButton.DpadLeft;

        public static Key ParseKey(string name, Key fallback)
        {
            return Enum.TryParse(name?.Trim(), true, out Key key) && key != Key.None ? key : fallback;
        }

        /// <summary>Gamepad button by name ("DpadRight", "North", ...); "None" or blank disables it.</summary>
        public static GamepadButton? ParseButton(string name, GamepadButton? fallback)
        {
            name = name?.Trim();
            if (string.IsNullOrEmpty(name) || name.Equals("None", StringComparison.OrdinalIgnoreCase))
            {
                return null;
            }
            return Enum.TryParse(name, true, out GamepadButton button) ? button : fallback;
        }

        public static bool AutopilotFlying()
        {
            var autopilot = Locator.GetShipBody()?.GetComponent<Autopilot>();
            return autopilot != null && autopilot.IsFlyingToDestination();
        }

        public static void Update()
        {
            if (!OWInput.IsInputMode(InputMode.ShipCockpit | InputMode.LandingCam))
            {
                return;
            }

            if (Pressed(CycleKey, CycleButton))
            {
                BetterAutopilot.ThrusterAxes = BetterAutopilot.ThrusterAxes % 3 + 1;
                BetterAutopilot.SaveSetting("thrusters", BetterAutopilot.ThrusterOptions[BetterAutopilot.ThrusterAxes - 1]);
                AutopilotPatches.ResumeOrientation();
                Notify($"AUTOPILOT MODE: {BetterAutopilot.ThrusterLabel.ToUpper()}");
            }

            if (AutopilotFlying() && !BetterAutopilot.GameDefault && Pressed(ToggleAlignKey, ToggleAlignButton))
            {
                BetterAutopilot.AutoOrient = !BetterAutopilot.AutoOrient;
                BetterAutopilot.SaveSetting("autoOrient", BetterAutopilot.AutoOrient);
                AutopilotPatches.ResumeOrientation();
                Notify(BetterAutopilot.AutoOrient ? "AUTO-ALIGN ON" : "AUTO-ALIGN OFF");
            }
        }

        private static bool Pressed(Key key, GamepadButton? button)
        {
            var keyboard = Keyboard.current;
            if (keyboard != null && keyboard[key].wasPressedThisFrame)
            {
                return true;
            }
            var gamepad = Gamepad.current;
            if (gamepad == null || button == null)
            {
                return false;
            }
            // Leave the D-pad to the scout camera while a tool is out.
            var tools = Locator.GetToolModeSwapper();
            if (tools != null && tools.GetToolMode() != ToolMode.None)
            {
                return false;
            }
            return gamepad[button.Value].wasPressedThisFrame;
        }

        private static void Notify(string text)
        {
            NotificationManager.SharedInstance?.PostNotification(new NotificationData(NotificationTarget.Ship, text, 2f));
        }
    }
}
