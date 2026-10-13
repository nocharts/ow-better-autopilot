using System.Collections.Generic;
using HarmonyLib;
using UnityEngine;

namespace BetterAutopilot
{
    /// <summary>
    /// The bodies the autopilot must not hit, plus the last-moment safety net:
    /// <see cref="CheckCollision"/> watches where the ship's current momentum is actually taking it.
    /// Route planning around predicted body motion lives in <see cref="PredictivePlanner"/>.
    /// </summary>
    public static class RoutePlanner
    {
        internal class Obstacle
        {
            public string Name;
            public AstroObject Astro;
            public OWRigidbody Body;
            public SunController Sun;
            public float HardRadius;
            // The destination is only checked for collisions; routing ends at its arrival distance.
            public bool IsTarget;

            public float CurrentHardRadius => Sun != null ? Sun.GetSurfaceRadius() : HardRadius;
            public float SafeRadius => Sun != null ? CurrentHardRadius * 1.6f + 200f : CurrentHardRadius * 1.25f + 50f;
            public float DangerRadius => Sun != null ? CurrentHardRadius * 1.3f + 100f : CurrentHardRadius * 1.15f + 40f;
        }

        private static readonly AccessTools.FieldRef<GravityVolume, float> UpperSurfaceRadius =
            AccessTools.FieldRefAccess<GravityVolume, float>("_upperSurfaceRadius");

        private static readonly List<Obstacle> _obstacles = new List<Obstacle>();

        internal static List<Obstacle> Obstacles => _obstacles;

        public static void Rebuild(ReferenceFrame target)
        {
            _obstacles.Clear();
            PredictivePlanner.Reset();
            var targetBody = target?.GetOWRigidBody();
            var sun = Object.FindObjectOfType<SunController>();

            foreach (var astro in Object.FindObjectsOfType<AstroObject>())
            {
                var body = astro.GetOWRigidbody();
                if (body == null || !astro.gameObject.activeInHierarchy)
                {
                    continue;
                }

                var obstacle = new Obstacle { Name = astro.name, Astro = astro, Body = body, IsTarget = body == targetBody };
                if (astro.GetAstroObjectName() == AstroObject.Name.Sun && sun != null)
                {
                    obstacle.Sun = sun;
                }
                else
                {
                    var gravity = astro.GetGravityVolume();
                    float radius = gravity != null ? UpperSurfaceRadius(gravity) : 0f;
                    if (radius < 20f)
                    {
                        // No usable gravity shell; fall back to the autopilot hover distance.
                        var rf = body.GetReferenceFrame();
                        radius = rf != null ? rf.GetAutopilotArrivalDistance() * 0.5f : 0f;
                    }
                    if (radius < 20f)
                    {
                        continue;
                    }
                    obstacle.HardRadius = radius;
                }

                _obstacles.Add(obstacle);
                BetterAutopilot.Log($"Obstacle {obstacle.Name}: radius {obstacle.CurrentHardRadius:F0}, safe {obstacle.SafeRadius:F0}"
                    + (obstacle.IsTarget ? " (target)" : ""));
            }
        }

        /// <summary>
        /// Projects the ship's current momentum relative to every body. A body is a threat when the
        /// course passes through its danger zone and we could no longer stop (at
        /// <paramref name="brakeAccel"/>, with a little reaction time) before reaching it.
        /// Returns the direction to thrust to get clear.
        /// </summary>
        public static bool CheckCollision(Vector3 shipPos, Vector3 shipVel, float brakeAccel,
            out Vector3 escape, out string name)
        {
            escape = Vector3.zero;
            name = null;
            float soonest = float.MaxValue;

            foreach (var obstacle in _obstacles)
            {
                if (obstacle.Body == null)
                {
                    continue;
                }
                Vector3 r = obstacle.Body.GetWorldCenterOfMass() - shipPos;
                Vector3 w = shipVel - obstacle.Body.GetVelocity();
                float danger = obstacle.DangerRadius;
                float dist = r.magnitude;
                float closing = Vector3.Dot(w, r) / Mathf.Max(dist, 0.001f);
                if (closing <= 0f)
                {
                    continue;
                }

                // Time and distance of closest approach on the current course.
                float t = Vector3.Dot(r, w) / Mathf.Max(w.sqrMagnitude, 1e-4f);
                Vector3 miss = w * t - r;
                bool inside = dist < danger;
                // Hovering or taking off near a surface is not an emergency; only a real descent is.
                if (inside && closing < 10f)
                {
                    continue;
                }
                // While the planner has a route it checked clear (gravity included), mostly trust it:
                // this straight-line projection cannot see the planned turn at a detour point. Still
                // step in if we would reach the danger zone within a couple of seconds.
                if (!inside && PredictivePlanner.RouteValidated && (dist - danger) / closing > 2.5f)
                {
                    continue;
                }
                if (!inside)
                {
                    // The normal braking plan already stops us short of the destination.
                    if (obstacle.IsTarget || miss.magnitude >= danger)
                    {
                        continue;
                    }
                    float stoppingDistance = closing * closing / (2f * Mathf.Max(brakeAccel, 0.1f)) + closing * 1.5f;
                    if (stoppingDistance < dist - danger)
                    {
                        continue;
                    }
                }
                if (t >= soonest)
                {
                    continue;
                }
                soonest = t;
                name = obstacle.Name;

                Vector3 wHat = w.normalized;
                Vector3 missHat = miss.sqrMagnitude > 1e-4f
                    ? miss.normalized
                    : Vector3.Cross(wHat, Mathf.Abs(wHat.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
                // The closer to a head-on hit, the more of the effort goes into braking.
                float headOn = 1f - Mathf.Clamp01(miss.magnitude / danger);
                escape = inside
                    ? (-r.normalized + missHat).normalized
                    : (missHat + -wHat * (0.5f + 1.5f * headOn)).normalized;
            }

            return name != null;
        }

        /// <summary>A body (not the target or the Sun) whose surface we are practically sitting on.</summary>
        internal static Obstacle LiftoffBody(Vector3 shipPos)
        {
            foreach (var obstacle in _obstacles)
            {
                if (obstacle.Body == null || obstacle.IsTarget || obstacle.Sun != null)
                {
                    continue;
                }
                if (Vector3.Distance(obstacle.Body.GetWorldCenterOfMass(), shipPos) < obstacle.CurrentHardRadius * 1.15f + 60f)
                {
                    return obstacle;
                }
            }
            return null;
        }

        /// <summary>How far from the centre counts as clear of a surface after lifting off.</summary>
        internal static float LiftoffClearRadius(Obstacle obstacle) => obstacle.CurrentHardRadius * 1.2f + 80f;

        /// <summary>For the debug log: the closest body (excluding the target) by distance to its surface.</summary>
        public static string DescribeNearest(Vector3 shipPos, Vector3 shipVel)
        {
            Obstacle nearest = null;
            float best = float.MaxValue;
            foreach (var obstacle in _obstacles)
            {
                if (obstacle.Body == null || obstacle.IsTarget)
                {
                    continue;
                }
                float surface = Vector3.Distance(obstacle.Body.GetWorldCenterOfMass(), shipPos) - obstacle.CurrentHardRadius;
                if (surface < best)
                {
                    best = surface;
                    nearest = obstacle;
                }
            }
            if (nearest == null)
            {
                return "";
            }
            Vector3 r = nearest.Body.GetWorldCenterOfMass() - shipPos;
            float closing = Vector3.Dot(shipVel - nearest.Body.GetVelocity(), r.normalized);
            return $", nearest {nearest.Name} {best:F0} m above surface (closing {closing:F0})";
        }
    }
}
