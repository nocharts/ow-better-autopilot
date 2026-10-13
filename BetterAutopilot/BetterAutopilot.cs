using System.Reflection;
using HarmonyLib;
using OWML.Common;
using OWML.ModHelper;

namespace BetterAutopilot
{
    public class BetterAutopilot : ModBehaviour
    {
        public static BetterAutopilot Instance;

        // Axes fired together: 2 or 3, or 1 = game default (the mod leaves the autopilot alone).
        public static int ThrusterAxes = 3;
        public static bool GameDefault => ThrusterAxes == 1;
        public static bool AutoOrient = true;
        public static bool DepartureView = true;
        public static bool ArrivalView;
        public static bool FaceTargetOnArrival = true;
        public static bool AvoidObstacles = true;
        public static bool BoostMatchVelocity = true;
        public static float MaxSpeed;
        public static float BrakingMargin = 0.95f;
        public static bool DebugLog;

        private void Awake()
        {
            Instance = this;
        }

        private void Start()
        {
            new Harmony(ModHelper.Manifest.UniqueName).PatchAll(Assembly.GetExecutingAssembly());
            ModHelper.Console.WriteLine("Better Autopilot ready", MessageType.Success);
        }

        private void Update() => Controls.Update();

        private void FixedUpdate() => ArrivalFacing.FixedUpdate();

        public override void Configure(IModConfig config)
        {
            var thrusters = config.GetSettingsValue<string>("thrusters") ?? "";
            // "Game default", "Fast (2 thrusters)", "Fastest (3 thrusters)" (older saves: "1 …", "2 …", "3 …").
            ThrusterAxes = thrusters.StartsWith("Game") || thrusters.StartsWith("1") ? 1
                : thrusters.StartsWith("2") || thrusters.StartsWith("Fast (") ? 2 : 3;
            AutoOrient = config.GetSettingsValue<bool>("autoOrient");
            DepartureView = config.GetSettingsValue<bool>("departureView");
            ArrivalView = config.GetSettingsValue<bool>("arrivalView");
            FaceTargetOnArrival = config.GetSettingsValue<bool>("faceTargetOnArrival");
            AvoidObstacles = config.GetSettingsValue<bool>("avoidObstacles");
            BoostMatchVelocity = config.GetSettingsValue<bool>("boostMatchVelocity");
            MaxSpeed = config.GetSettingsValue<float>("maxSpeed");
            BrakingMargin = config.GetSettingsValue<float>("brakingMargin");
            DebugLog = config.GetSettingsValue<bool>("debugLog");
            Controls.CycleKey = Controls.ParseKey(config.GetSettingsValue<string>("cycleThrustersKey"), UnityEngine.InputSystem.Key.J);
            Controls.ToggleAlignKey = Controls.ParseKey(config.GetSettingsValue<string>("alignKey"), UnityEngine.InputSystem.Key.G);
            Controls.CycleButton = Controls.ParseButton(config.GetSettingsValue<string>("cycleThrustersButton"), UnityEngine.InputSystem.LowLevel.GamepadButton.DpadRight);
            Controls.ToggleAlignButton = Controls.ParseButton(config.GetSettingsValue<string>("alignButton"), UnityEngine.InputSystem.LowLevel.GamepadButton.DpadLeft);
        }

        public static readonly string[] ThrusterOptions = { "Game default", "Fast (2 thrusters)", "Fastest (3 thrusters)" };

        public static string ThrusterLabel => GameDefault ? "Game Default" : ThrusterAxes == 2 ? "Fast" : "Fastest";

        /// <summary>Writes a hotkey change back to config.json so it sticks.</summary>
        public static void SaveSetting(string key, object value)
        {
            if (Instance == null)
            {
                return;
            }
            var helper = Instance.ModHelper;
            helper.Config.SetSettingsValue(key, value);
            helper.Storage.Save(helper.Config, "config.json");
        }

        public static void Log(string message)
        {
            if (DebugLog)
            {
                Instance?.ModHelper.Console.WriteLine(message, MessageType.Info);
            }
        }

        // Largest input magnitude the per-axis clamp allows for the chosen axis count.
        public static float ThrustCap => ThrusterAxes == 3 ? 1.7320508f : ThrusterAxes == 2 ? 1.4142135f : 1f;
    }
}
