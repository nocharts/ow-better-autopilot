using System.Diagnostics;
using System.Linq;

namespace BetterAutopilot
{
    /// <summary>Extra debug-log detail for working out what went wrong after a crash.</summary>
    public static class Diagnostics
    {
        private static ImpactSensor _sensor;

        /// <summary>Logs every ship impact (a new ship every loop, so re-hook when it changes).</summary>
        public static void WatchImpacts(Autopilot autopilot)
        {
            var sensor = autopilot.GetComponent<ImpactSensor>();
            if (sensor == null || sensor == _sensor)
            {
                return;
            }
            sensor.OnImpact += impact =>
            {
                // The player bumping about inside the cabin is not interesting.
                if (impact.otherBody != null && impact.otherBody == Locator.GetPlayerBody())
                {
                    return;
                }
                string other = impact.otherCollider != null ? impact.otherCollider.name : "?";
                string body = impact.otherBody != null ? impact.otherBody.name : "?";
                BetterAutopilot.Log($"Ship impact at {impact.speed:F0} m/s with {other} ({body}); autopilot flying {autopilot.IsFlyingToDestination()}, matching {autopilot.IsMatchingVelocity()}");
            };
            _sensor = sensor;
        }

        /// <summary>The game method that called into the patched method (skipping Harmony/our frames).</summary>
        public static string Caller()
        {
            var frames = new StackTrace(2, false).GetFrames() ?? new StackFrame[0];
            var names = frames
                .Select(f => f.GetMethod())
                .Where(m => m != null && m.DeclaringType != null && !m.DeclaringType.FullName.StartsWith("BetterAutopilot")
                    && !m.DeclaringType.FullName.StartsWith("HarmonyLib") && !m.Name.Contains("DMD"))
                .Take(3)
                .Select(m => $"{m.DeclaringType.Name}.{m.Name}");
            return string.Join(" <- ", names);
        }
    }
}
