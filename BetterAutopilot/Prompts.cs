using System;
using HarmonyLib;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;

namespace BetterAutopilot
{
    /// <summary>
    /// Adds our hotkeys to the cockpit's upper-left prompt list, under the game's own
    /// (Engage Autopilot, Match Velocity...). Custom-sprite prompts cannot change their icon, so each
    /// action has a keyboard and a gamepad prompt and we show the one matching the active device.
    /// </summary>
    [HarmonyPatch(typeof(ShipPromptController))]
    public static class Prompts
    {
        private static readonly AccessTools.FieldRef<ShipPromptController, ShipCockpitController> FlightConsole =
            AccessTools.FieldRefAccess<ShipPromptController, ShipCockpitController>("_flightConsole");
        private static readonly AccessTools.FieldRef<ShipPromptController, LandingPadManager> LandingPads =
            AccessTools.FieldRefAccess<ShipPromptController, LandingPadManager>("_landingPadManager");
        private static readonly AccessTools.FieldRef<ShipPromptController, bool> ShipDestroyed =
            AccessTools.FieldRefAccess<ShipPromptController, bool>("_shipDestroyed");

        private class PromptPair
        {
            public ScreenPrompt Keyboard;
            public ScreenPrompt Gamepad;
            public Key Key;
            public GamepadButton? Button;
        }

        private static PromptPair _cycle;
        private static PromptPair _align;
        private static bool _subscribed;

        [HarmonyPostfix]
        [HarmonyPatch(nameof(ShipPromptController.LateInitialize))]
        public static void LateInitialize_Postfix()
        {
            _cycle = Create(Controls.CycleKey, Controls.CycleButton);
            _align = Create(Controls.ToggleAlignKey, Controls.ToggleAlignButton);
            if (!_subscribed)
            {
                // Controller icon style (PlayStation / Xbox...) changed: rebuild the gamepad prompts.
                ButtonPromptLibrary.OnUpdateButtonPromptConfig += RebuildGamepadPrompts;
                _subscribed = true;
            }
        }

        [HarmonyPostfix]
        [HarmonyPatch("Update")]
        public static void Update_Postfix(ShipPromptController __instance)
        {
            if (_cycle == null || _align == null)
            {
                return;
            }
            // Picks up key/button changes made in the mod settings.
            if (_cycle.Key != Controls.CycleKey || _cycle.Button != Controls.CycleButton
                || _align.Key != Controls.ToggleAlignKey || _align.Button != Controls.ToggleAlignButton)
            {
                Remove(_cycle);
                Remove(_align);
                LateInitialize_Postfix();
            }

            bool cockpit = OWInput.IsInputMode(InputMode.ShipCockpit)
                && !ShipDestroyed(__instance)
                && !LandingPads(__instance).IsLanded();
            bool flying = Controls.AutopilotFlying();

            bool showCycle = cockpit && (flying || FlightConsole(__instance).IsAutopilotAvailable());
            bool showAlign = cockpit && flying && !BetterAutopilot.GameDefault;

            SetText(_cycle, $"Autopilot Mode: {BetterAutopilot.ThrusterLabel}");
            SetText(_align, BetterAutopilot.AutoOrient ? "Auto-Align: On" : "Auto-Align: Off");
            Show(_cycle, showCycle);
            Show(_align, showAlign);
        }

        private static PromptPair Create(Key key, GamepadButton? button)
        {
            var pair = new PromptPair { Key = key, Button = button };
            pair.Keyboard = Add(KeyboardSprite(key));
            pair.Gamepad = button != null ? Add(GamepadSprite(button.Value)) : null;
            return pair;
        }

        private static ScreenPrompt Add(Sprite sprite)
        {
            if (sprite == null)
            {
                return null;
            }
            var prompt = new ScreenPrompt("<CMD>   ", sprite);
            Locator.GetPromptManager().AddScreenPrompt(prompt, PromptPosition.UpperLeft);
            return prompt;
        }

        private static void Remove(PromptPair pair)
        {
            var manager = Locator.GetPromptManager();
            if (manager == null)
            {
                return;
            }
            if (pair.Keyboard != null)
            {
                manager.RemoveScreenPrompt(pair.Keyboard, PromptPosition.UpperLeft);
            }
            if (pair.Gamepad != null)
            {
                manager.RemoveScreenPrompt(pair.Gamepad, PromptPosition.UpperLeft);
            }
        }

        private static void RebuildGamepadPrompts()
        {
            var manager = Locator.GetPromptManager();
            if (manager == null)
            {
                return;
            }
            foreach (var pair in new[] { _cycle, _align })
            {
                if (pair?.Gamepad == null || pair.Button == null)
                {
                    continue;
                }
                manager.RemoveScreenPrompt(pair.Gamepad, PromptPosition.UpperLeft);
                pair.Gamepad = Add(GamepadSprite(pair.Button.Value));
            }
        }

        private static void SetText(PromptPair pair, string text)
        {
            pair.Keyboard?.SetText("<CMD>   " + text);
            pair.Gamepad?.SetText("<CMD>   " + text);
        }

        private static void Show(PromptPair pair, bool visible)
        {
            // Custom-sprite prompts skip the game's "show prompts" option, so check it ourselves.
            visible &= PlayerData.GetPromptsEnabled();
            bool gamepad = OWInput.UsingGamepad();
            // The D-pad belongs to the scout camera while a tool is out, so the button does nothing then.
            var tools = Locator.GetToolModeSwapper();
            bool gamepadBlocked = tools != null && tools.GetToolMode() != ToolMode.None;
            pair.Keyboard?.SetVisibility(visible && !gamepad);
            pair.Gamepad?.SetVisibility(visible && gamepad && !gamepadBlocked);
        }

        private static Sprite KeyboardSprite(Key key) => ToSprite(ButtonPromptLibrary.SharedInstance.GetButtonTexture(ToKeyCode(key)));

        private static Sprite GamepadSprite(GamepadButton button)
        {
            var joystick = ToJoystickButton(button);
            return joystick == JoystickButton.None ? null : ToSprite(ButtonPromptLibrary.SharedInstance.GetButtonTexture(joystick));
        }

        private static Sprite ToSprite(Texture2D texture)
        {
            return texture == null
                ? null
                : Sprite.Create(texture, new Rect(0f, 0f, texture.width, texture.height), new Vector2(0.5f, 0.5f));
        }

        // The prompt icons are indexed by the legacy KeyCode, the hotkeys by the new Input System Key.
        private static KeyCode ToKeyCode(Key key)
        {
            if (key >= Key.A && key <= Key.Z)
            {
                return KeyCode.A + (key - Key.A);
            }
            if (key >= Key.Digit1 && key <= Key.Digit9)
            {
                return KeyCode.Alpha1 + (key - Key.Digit1);
            }
            if (key == Key.Digit0)
            {
                return KeyCode.Alpha0;
            }
            if (key >= Key.F1 && key <= Key.F12)
            {
                return KeyCode.F1 + (key - Key.F1);
            }
            if (key >= Key.Numpad0 && key <= Key.Numpad9)
            {
                return KeyCode.Keypad0 + (key - Key.Numpad0);
            }
            return Enum.TryParse(key.ToString(), true, out KeyCode code) ? code : KeyCode.None;
        }

        private static JoystickButton ToJoystickButton(GamepadButton button)
        {
            switch (button)
            {
                case GamepadButton.DpadUp: return JoystickButton.DPadUp;
                case GamepadButton.DpadDown: return JoystickButton.DPadDown;
                case GamepadButton.DpadLeft: return JoystickButton.DPadLeft;
                case GamepadButton.DpadRight: return JoystickButton.DPadRight;
                case GamepadButton.North: return JoystickButton.FaceUp;
                case GamepadButton.South: return JoystickButton.FaceDown;
                case GamepadButton.West: return JoystickButton.FaceLeft;
                case GamepadButton.East: return JoystickButton.FaceRight;
                case GamepadButton.LeftShoulder: return JoystickButton.LeftBumper;
                case GamepadButton.RightShoulder: return JoystickButton.RightBumper;
                case GamepadButton.LeftTrigger: return JoystickButton.LeftTrigger;
                case GamepadButton.RightTrigger: return JoystickButton.RightTrigger;
                case GamepadButton.LeftStick: return JoystickButton.LeftStickClick;
                case GamepadButton.RightStick: return JoystickButton.RightStickClick;
                case GamepadButton.Start: return JoystickButton.Start;
                case GamepadButton.Select: return JoystickButton.Select;
                default: return JoystickButton.None;
            }
        }
    }
}
