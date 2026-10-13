using System.Collections.Generic;
using UnityEngine;

namespace BetterAutopilot
{
    /// <summary>
    /// Plans the route ahead of time instead of reacting to what is nearby:
    /// <list type="number">
    /// <item>Projects every body forward along its orbit (round its primary: the Sun, a planet, the
    /// Twins' focal point).</item>
    /// <item>Simulates the ship flying the route (accelerate, then brake) relative to the target and
    /// checks the clearance to each body at the moment the ship actually gets there.</item>
    /// <item>On a conflict, tries a ring of detour points round the predicted meeting point and keeps
    /// the quickest clear one.</item>
    /// </list>
    /// Waypoints are fixed relative to the target and only replaced when the route stops being clear,
    /// which keeps the flight steady.
    /// </summary>
    public static class PredictivePlanner
    {
        private const float ReplanInterval = 1.5f;
        private const int Steps = 400;
        private static readonly float[] DetourScales = { 1.35f, 1.9f, 2.8f };
        // Small pushes of the closest approach point straight out from the body, for routes that only graze.
        private static readonly float[] NudgeScales = { 1.1f, 1.25f, 1.45f };
        private const int DetourAngles = 8;

        private static bool _hasWaypoint;
        private static Vector3 _waypoint; // Relative to the target.
        private static Vector3 _approachDir; // Direction we were heading towards the waypoint when it was chosen.
        private static string _blocker;
        private static float _nextReplan;

        // Scratch state for one planning pass.
        private static float _dt;
        private static float _accel;       // Planning acceleration (braking curve).
        private static float _maxThrust;   // Thrust limit in the flight simulation.
        private static float _arrival;
        private static Vector3 _shipStart; // Relative to the target.
        private static Vector3 _velStart;
        private static readonly Dictionary<AstroObject, Vector3[]> _tracks = new Dictionary<AstroObject, Vector3[]>();
        private static readonly HashSet<AstroObject> _building = new HashSet<AstroObject>();
        private static readonly List<RelativeTrack> _relTracks = new List<RelativeTrack>();
        private static readonly Vector3[] _samples = new Vector3[Steps];
        // Gravity the simulated ship has to fight: the Sun's (which pulls on the ship and the target
        // differently) and the target's own.
        private static Vector3[] _targetTrack;
        private static Vector3[] _sunTrack;
        private static GravityVolume _sunGravity;
        private static GravityVolume _targetGravity;

        /// <summary>True while the route being flown was checked clear (with gravity) on the last replan.</summary>
        public static bool RouteValidated { get; private set; }

        private class RelativeTrack
        {
            public string Name;
            public Vector3[] Positions; // Relative to the target at each step.
            public float Clearance;
            public float DetourRadius;  // How far out to place detour points (the full safety zone).
        }

        private class Conflict
        {
            public string Name;
            public Vector3 ObstaclePos;
            public Vector3 ShipPos;
            public Vector3 PathDir;
            public float DetourRadius;
        }

        public static void Reset()
        {
            _hasWaypoint = false;
            _blocker = null;
            _nextReplan = 0f;
            RouteValidated = false;
        }

        /// <summary>
        /// Where to fly now. <paramref name="pathLength"/> is the remaining route length to the target
        /// centre (the caller subtracts the arrival distance). Returns true while detouring.
        /// </summary>
        public static bool Follow(Vector3 shipPos, Vector3 relVel, ReferenceFrame target, float arrival, float accel,
            out Vector3 aim, out float pathLength, out string blocker)
        {
            Vector3 ship = shipPos - target.GetPosition();

            // Passed once we cross the plane through the waypoint, square to how we approached it.
            if (_hasWaypoint && Vector3.Dot(ship - _waypoint, _approachDir) >= 0f)
            {
                BetterAutopilot.Log($"Route: passed the waypoint around {_blocker}");
                _hasWaypoint = false;
                _nextReplan = 0f;
            }
            if (Time.time >= _nextReplan)
            {
                Replan(ship, relVel, target, arrival, accel);
                _nextReplan = Time.time + ReplanInterval;
            }

            if (_hasWaypoint)
            {
                Vector3 toWaypoint = _waypoint - ship;
                aim = toWaypoint.normalized;
                pathLength = toWaypoint.magnitude + (EndPoint(_waypoint, arrival) - _waypoint).magnitude + arrival;
                blocker = _blocker;
                return true;
            }
            aim = (-ship).normalized;
            pathLength = ship.magnitude;
            blocker = null;
            return false;
        }

        private static void Replan(Vector3 ship, Vector3 relVel, ReferenceFrame target, float arrival, float accel)
        {
            _accel = Mathf.Max(accel, 1f);
            _maxThrust = _accel / Mathf.Max(BetterAutopilot.BrakingMargin, 0.1f);
            _arrival = arrival;
            _shipStart = ship;
            _velStart = relVel;
            // Horizon: the direct trip with room for a detour and for killing our current speed.
            float tripTime = EstimateTime(ship.magnitude * 1.5f + 2000f) + 2f * relVel.magnitude / _accel;
            _dt = Mathf.Max(0.05f, tripTime * 1.2f / Steps);
            if (!BuildTracks(ship, target, arrival))
            {
                return;
            }

            // Leaving a detour for the direct line needs extra margin, so borderline passes do not flip-flop.
            if (Check(false, Vector3.zero, out _, out _, _hasWaypoint ? 1.15f : 1f) == null)
            {
                if (_hasWaypoint)
                {
                    BetterAutopilot.Log("Route: direct line is clear now");
                }
                _hasWaypoint = false;
                _blocker = null;
                RouteValidated = true;
                return;
            }

            // Keep the current detour while it is still clear, so the route does not wander.
            if (_hasWaypoint && Check(true, _waypoint, out _, out _) == null)
            {
                RouteValidated = true;
                return;
            }

            var conflict = (_hasWaypoint ? Check(true, _waypoint, out _, out _) : null) ?? Check(false, Vector3.zero, out _, out _);
            Vector3 n1 = Vector3.Cross(conflict.PathDir, Mathf.Abs(conflict.PathDir.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
            Vector3 n2 = Vector3.Cross(conflict.PathDir, n1);

            var candidates = new List<Vector3>();
            // Nudges: push the point where the route grazes the body just far enough out.
            Vector3 outward = conflict.ShipPos - conflict.ObstaclePos;
            outward -= conflict.PathDir * Vector3.Dot(outward, conflict.PathDir);
            if (outward.sqrMagnitude > 1f)
            {
                outward.Normalize();
                foreach (float scale in NudgeScales)
                {
                    candidates.Add(conflict.ObstaclePos + outward * conflict.DetourRadius * scale);
                }
            }
            // Rings: every direction round the body (over, under, either side), at a few sizes.
            foreach (float scale in DetourScales)
            {
                for (int i = 0; i < DetourAngles; i++)
                {
                    float angle = i * Mathf.PI * 2f / DetourAngles;
                    candidates.Add(conflict.ObstaclePos + (n1 * Mathf.Cos(angle) + n2 * Mathf.Sin(angle)) * conflict.DetourRadius * scale);
                }
            }

            // Fly every candidate and keep the quickest clear one.
            Vector3 best = Vector3.zero;
            float bestTime = float.MaxValue;
            Vector3 fallback = Vector3.zero;
            float fallbackRatio = -1f;
            foreach (var candidate in candidates)
            {
                if (Check(true, candidate, out float time, out float ratio) == null)
                {
                    if (time < bestTime)
                    {
                        bestTime = time;
                        best = candidate;
                    }
                }
                else if (ratio > fallbackRatio)
                {
                    fallbackRatio = ratio;
                    fallback = candidate;
                }
            }

            _hasWaypoint = true;
            _blocker = conflict.Name;
            RouteValidated = bestTime < float.MaxValue;
            _waypoint = bestTime < float.MaxValue ? best : fallback;
            _approachDir = (_waypoint - ship).normalized;
            if (bestTime < float.MaxValue)
            {
                BetterAutopilot.Log($"Route: detour around {conflict.Name}, about {bestTime:F0} s to arrival");
            }
            else
            {
                BetterAutopilot.Log($"Route: no fully clear detour around {conflict.Name}; best clearance {fallbackRatio * 100f:F0}% (collision check stays armed)");
            }
        }

        /// <summary>Simulates the flight (direct or via a waypoint) and returns the first clearance violation, if any.</summary>
        private static Conflict Check(bool viaWaypoint, Vector3 waypoint, out float time, out float worstRatio, float clearanceScale = 1f)
        {
            int count = Simulate(viaWaypoint, waypoint, out time);
            worstRatio = float.MaxValue;
            Conflict first = null;
            int firstStep = int.MaxValue;
            foreach (var track in _relTracks)
            {
                for (int k = 0; k < count && k < firstStep; k++)
                {
                    // Compare against the body a couple of steps either side, to absorb timing error.
                    float d = float.MaxValue;
                    for (int j = Mathf.Max(0, k - 2); j <= Mathf.Min(Steps - 1, k + 2); j++)
                    {
                        d = Mathf.Min(d, Vector3.Distance(_samples[k], track.Positions[j]));
                    }
                    float clearance = track.Clearance * clearanceScale;
                    worstRatio = Mathf.Min(worstRatio, d / clearance);
                    if (d < clearance)
                    {
                        firstStep = k;
                        Vector3 dir = k + 1 < count ? _samples[k + 1] - _samples[k] : -_shipStart;
                        first = new Conflict
                        {
                            Name = track.Name,
                            ObstaclePos = track.Positions[k],
                            ShipPos = _samples[k],
                            PathDir = dir.sqrMagnitude > 1e-6f ? dir.normalized : (-_shipStart).normalized,
                            DetourRadius = track.DetourRadius,
                        };
                        break;
                    }
                }
            }
            return first;
        }

        /// <summary>
        /// Flies the autopilot's own control law forward from the ship's real position and velocity
        /// (relative to the target), recording the position at each step. Gravity is left out: the
        /// real autopilot cancels it with feed-forward.
        /// </summary>
        private static int Simulate(bool viaWaypoint, Vector3 waypoint, out float time)
        {
            Vector3 p = _shipStart;
            Vector3 v = _velStart;
            Vector3 approach = (waypoint - p).normalized;
            float lag = _accel * AutopilotPatches.ResponseTime;
            int substeps = Mathf.Max(1, Mathf.CeilToInt(_dt / 0.1f));
            float h = _dt / substeps;

            for (int k = 0; k < Steps; k++)
            {
                _samples[k] = p;
                for (int i = 0; i < substeps; i++)
                {
                    if (viaWaypoint && Vector3.Dot(p - waypoint, approach) >= 0f)
                    {
                        viaWaypoint = false;
                    }
                    float dist = p.magnitude;
                    if (dist - _arrival <= AutopilotPatches.StopBuffer + 5f)
                    {
                        time = k * _dt + i * h;
                        return k + 1;
                    }

                    Vector3 aim;
                    float remaining;
                    if (viaWaypoint)
                    {
                        Vector3 toWaypoint = waypoint - p;
                        aim = toWaypoint.normalized;
                        remaining = toWaypoint.magnitude + (EndPoint(waypoint, _arrival) - waypoint).magnitude;
                    }
                    else
                    {
                        aim = -p / dist;
                        remaining = dist - _arrival;
                    }
                    float room = Mathf.Max(0f, remaining - AutopilotPatches.StopBuffer);
                    float speed = Mathf.Sqrt(2f * _accel * room + lag * lag) - lag;
                    if (BetterAutopilot.MaxSpeed > 0f)
                    {
                        speed = Mathf.Min(speed, BetterAutopilot.MaxSpeed);
                    }
                    // Same as the real autopilot: try to cancel gravity, with only so much thrust.
                    Vector3 gravity = RelativeGravity(p, k);
                    Vector3 thrust = (aim * speed - v) / AutopilotPatches.VelocityTimeConstant - gravity;
                    if (thrust.magnitude > _maxThrust)
                    {
                        thrust = thrust.normalized * _maxThrust;
                    }
                    v += (thrust + gravity) * h;
                    p += v * h;
                }
            }
            // Did not arrive within the horizon.
            time = Steps * _dt + EstimateTime(p.magnitude);
            return Steps;
        }

        /// <summary>Gravitational acceleration of the ship relative to the target at step k.</summary>
        private static Vector3 RelativeGravity(Vector3 shipRel, int k)
        {
            Vector3 g = Vector3.zero;
            if (_sunGravity != null)
            {
                Vector3 toSunFromShip = _sunTrack[k] - (shipRel + _targetTrack[k]);
                Vector3 toSunFromTarget = _sunTrack[k] - _targetTrack[k];
                g += toSunFromShip.normalized * _sunGravity.CalculateGravityMagnitude(toSunFromShip.magnitude)
                    - toSunFromTarget.normalized * _sunGravity.CalculateGravityMagnitude(toSunFromTarget.magnitude);
            }
            if (_targetGravity != null)
            {
                float d = shipRel.magnitude;
                g += -shipRel / Mathf.Max(d, 1f) * _targetGravity.CalculateGravityMagnitude(d);
            }
            return g;
        }

        // Arrival point on the side we come in from.
        private static Vector3 EndPoint(Vector3 from, float arrival) => from.normalized * arrival;

        // Rest-to-rest time over a distance at the planning acceleration.
        private static float EstimateTime(float distance) => 2f * Mathf.Sqrt(Mathf.Max(distance, 0f) / _accel);

        private static bool BuildTracks(Vector3 ship, ReferenceFrame target, float arrival)
        {
            _tracks.Clear();
            _relTracks.Clear();

            Vector3[] targetTrack;
            var targetAstro = target.GetAstroObject();
            if (targetAstro != null && targetAstro.GetOWRigidbody() != null)
            {
                // The reference point can sit off the body's centre (e.g. a station on a planet).
                Vector3 offset = target.GetPosition() - targetAstro.GetOWRigidbody().GetWorldCenterOfMass();
                var bodyTrack = Track(targetAstro);
                targetTrack = new Vector3[Steps];
                for (int k = 0; k < Steps; k++)
                {
                    targetTrack[k] = bodyTrack[k] + offset;
                }
            }
            else
            {
                targetTrack = Linear(target.GetPosition(), target.GetVelocity());
            }
            _targetTrack = targetTrack;
            _targetGravity = targetAstro != null ? targetAstro.GetGravityVolume() : null;
            _sunGravity = null;
            _sunTrack = null;
            foreach (var obstacle in RoutePlanner.Obstacles)
            {
                if (obstacle.Sun != null && obstacle.Astro != null && !obstacle.IsTarget)
                {
                    _sunGravity = obstacle.Astro.GetGravityVolume();
                    _sunTrack = Track(obstacle.Astro);
                }
            }

            foreach (var obstacle in RoutePlanner.Obstacles)
            {
                if (obstacle.Body == null || obstacle.Astro == null)
                {
                    continue;
                }
                var track = Track(obstacle.Astro);
                var rel = new Vector3[Steps];
                for (int k = 0; k < Steps; k++)
                {
                    rel[k] = track[k] - targetTrack[k];
                }

                float clearance;
                float detourRadius = obstacle.SafeRadius;
                if (obstacle.IsTarget)
                {
                    // Only guard the surface; the approach itself stops at the arrival distance.
                    clearance = Mathf.Min(obstacle.CurrentHardRadius * 1.15f, arrival * 0.9f);
                }
                else
                {
                    // Bodies wholly inside the arrival zone are never reached.
                    if (rel[0].magnitude + obstacle.CurrentHardRadius < arrival)
                    {
                        continue;
                    }
                    clearance = obstacle.SafeRadius;
                    // Departing from inside the safety shell: just never get closer than we are now
                    // (we may even be below the estimated surface radius, e.g. in a valley).
                    float startDistance = Vector3.Distance(ship, rel[0]);
                    if (startDistance < clearance)
                    {
                        clearance = Mathf.Min(obstacle.CurrentHardRadius * 1.1f, startDistance * 0.97f);
                    }
                }
                // Detours always go out to the full safety zone, so leaving a planet climbs away from
                // the ground instead of skimming round it.
                _relTracks.Add(new RelativeTrack { Name = obstacle.Name, Positions = rel, Clearance = clearance, DetourRadius = Mathf.Max(detourRadius, clearance) });
            }
            return true;
        }

        /// <summary>
        /// Predicted world position of a body at each step. Near-circular orbits are rotated round the
        /// primary at their current angular rate (exact for circular orbits, and stable); eccentric ones
        /// (the Interloper) are integrated with the primary's own gravity function.
        /// </summary>
        private static Vector3[] Track(AstroObject astro)
        {
            if (_tracks.TryGetValue(astro, out var cached))
            {
                return cached;
            }
            _building.Add(astro);

            var body = astro.GetOWRigidbody();
            Vector3 p0 = body.GetWorldCenterOfMass();
            Vector3 v0 = body.GetVelocity();
            var primary = astro.GetPrimaryBody();
            var primaryBody = primary != null ? primary.GetOWRigidbody() : null;

            Vector3[] positions;
            if (primary == null || primary == astro || primaryBody == null || _building.Contains(primary))
            {
                positions = Linear(p0, v0);
            }
            else
            {
                var primaryTrack = Track(primary);
                Vector3 r = p0 - primaryBody.GetWorldCenterOfMass();
                Vector3 v = v0 - primaryBody.GetVelocity();
                var gravity = primary.GetGravityVolume();
                positions = new Vector3[Steps];
                float radialShare = v.sqrMagnitude > 1e-4f ? Mathf.Abs(Vector3.Dot(v.normalized, r.normalized)) : 0f;

                if (radialShare > 0.05f && gravity != null)
                {
                    const int substeps = 4;
                    float h = _dt / substeps;
                    for (int k = 0; k < Steps; k++)
                    {
                        positions[k] = primaryTrack[k] + r;
                        for (int i = 0; i < substeps; i++)
                        {
                            float dist = r.magnitude;
                            v += -r / Mathf.Max(dist, 1f) * gravity.CalculateGravityMagnitude(dist) * h;
                            r += v * h;
                        }
                    }
                }
                else
                {
                    Vector3 omega = Vector3.Cross(r, v) / Mathf.Max(r.sqrMagnitude, 1f);
                    float rate = omega.magnitude;
                    Vector3 axis = rate > 1e-9f ? omega / rate : Vector3.up;
                    for (int k = 0; k < Steps; k++)
                    {
                        positions[k] = primaryTrack[k] + Quaternion.AngleAxis(rate * k * _dt * Mathf.Rad2Deg, axis) * r;
                    }
                }
            }

            _building.Remove(astro);
            _tracks[astro] = positions;
            return positions;
        }

        private static Vector3[] Linear(Vector3 p0, Vector3 v0)
        {
            var positions = new Vector3[Steps];
            for (int k = 0; k < Steps; k++)
            {
                positions[k] = p0 + v0 * (k * _dt);
            }
            return positions;
        }
    }
}
