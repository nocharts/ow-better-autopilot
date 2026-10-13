using HarmonyLib;
using UnityEngine;

namespace BetterAutopilot
{
    [HarmonyPatch(typeof(Autopilot))]
    public static class AutopilotPatches
    {
        private static readonly AccessTools.FieldRef<Autopilot, ReferenceFrame> RefFrame =
            AccessTools.FieldRefAccess<Autopilot, ReferenceFrame>("_referenceFrame");
        private static readonly AccessTools.FieldRef<Autopilot, OWRigidbody> Body =
            AccessTools.FieldRefAccess<Autopilot, OWRigidbody>("_owRigidbody");
        private static readonly AccessTools.FieldRef<Autopilot, ForceDetector> Forces =
            AccessTools.FieldRefAccess<Autopilot, ForceDetector>("_forceDetector");
        private static readonly AccessTools.FieldRef<Autopilot, ShipResources> Resources =
            AccessTools.FieldRefAccess<Autopilot, ShipResources>("_shipResources");
        private static readonly AccessTools.FieldRef<Autopilot, bool> IsShipAutopilot =
            AccessTools.FieldRefAccess<Autopilot, bool>("_isShipAutopilot");
        private static readonly AccessTools.FieldRef<Autopilot, bool> LiningUp =
            AccessTools.FieldRefAccess<Autopilot, bool>("_isLiningUpDestination");
        private static readonly AccessTools.FieldRef<Autopilot, bool> Approaching =
            AccessTools.FieldRefAccess<Autopilot, bool>("_isApproachingDestination");
        private static readonly AccessTools.FieldRef<ThrusterController, ThrusterModel> Model =
            AccessTools.FieldRefAccess<ThrusterController, ThrusterModel>("_thrusterModel");

        // Seconds of lag (turning, thruster spool) the braking plan allows for.
        internal const float ResponseTime = 0.75f;
        // How quickly velocity errors are corrected.
        internal const float VelocityTimeConstant = 0.5f;
        private const float MaxTurnRate = 0.8f;
        // Auto-align pauses while the player steers, and this long after they let go.
        private const float PlayerOverrideSeconds = 1f;
        // Metres short of the arrival distance the braking plan aims to stop at.
        // Must stay under 50 m, or the game reports the arrival as an undershoot.
        internal const float StopBuffer = 25f;
        private const float EvadeHoldSeconds = 1f;

        private static bool _braking;
        // View mode: line the forward + up diagonal up with the target (thrust runs either way along it,
        // so braking needs no turn). Off: whichever diagonal needs the least turning.
        private static bool _viewActive;
        // Sticky once the ship starts slowing for arrival, so departure view cannot flip back on.
        private static bool _decelerating;
        // What auto-align points the chosen diagonal along (the line of travel, normally).
        private static Vector3 _alignDirWorld;
        // Climbing straight off a surface before starting the route.
        private static RoutePlanner.Obstacle _liftoffBody;
        private static bool _liftoffDone;
        private const float LiftoffClimbSpeed = 60f;
        // The Quantum Moon jumps away (and the game drops the target) whenever nobody is looking at
        // it, so for that target we keep the nose on it instead of using a thruster diagonal.
        private static bool _keepTargetInView;
        private static Vector3 _targetDirWorld;
        private static bool _handledThisFrame;
        private static string _lastPhase;
        private static Vector3 _desiredThrustWorld;
        private static Vector3 _diagonal;
        private static float _playerRotateTime = -100f;
        private static float _lastTelemetryTime = -100f;
        private static float _evadeUntil = -100f;
        private static Vector3 _escape;
        private static string _threat;

        [HarmonyPostfix]
        [HarmonyPatch(nameof(Autopilot.FlyToDestination))]
        public static void FlyToDestination_Postfix(Autopilot __instance, bool __result, ReferenceFrame referenceFrame)
        {
            if (!__result || !IsShipAutopilot(__instance))
            {
                return;
            }
            _braking = false;
            _viewActive = false;
            _decelerating = false;
            _alignDirWorld = Vector3.zero;
            _liftoffBody = null;
            _liftoffDone = false;
            _evadeUntil = -100f;
            _lastPhase = null;
            _desiredThrustWorld = Vector3.zero;
            _diagonal = Vector3.zero;
            _keepTargetInView = referenceFrame.GetAstroObject() != null
                && referenceFrame.GetAstroObject().GetAstroObjectName() == AstroObject.Name.QuantumMoon;
            if (_keepTargetInView)
            {
                BetterAutopilot.Log("Target is the Quantum Moon: keeping it in view (single-thruster speed)");
            }
            ArrivalFacing.Watch(__instance, referenceFrame);
            Diagnostics.WatchImpacts(__instance);
            RoutePlanner.Rebuild(referenceFrame);
            BetterAutopilot.Log($"Autopilot to {referenceFrame.GetHUDDisplayName()}, arrival distance {referenceFrame.GetAutopilotArrivalDistance():F0}");
        }

        [HarmonyPrefix]
        [HarmonyPatch("ReadTranslationalInput")]
        public static bool ReadTranslationalInput_Prefix(Autopilot __instance, ref Vector3 __result)
        {
            _handledThisFrame = false;
            if (BetterAutopilot.GameDefault
                || !IsShipAutopilot(__instance)
                || !__instance.IsFlyingToDestination()
                || __instance.IsMatchingVelocity()
                || RefFrame(__instance) == null
                || !Resources(__instance).AreThrustersUsable())
            {
                // Vanilla handles aborts, match velocity and the final arrival.
                _desiredThrustWorld = Vector3.zero;
                return true;
            }

            __result = FlyStep(__instance);
            _handledThisFrame = true;
            return false;
        }

        [HarmonyPostfix]
        [HarmonyPatch("ReadTranslationalInput")]
        public static void ReadTranslationalInput_Postfix(Autopilot __instance, ref Vector3 __result)
        {
            if (_handledThisFrame || !BetterAutopilot.BoostMatchVelocity || BetterAutopilot.GameDefault
                || !IsShipAutopilot(__instance) || !__instance.IsMatchingVelocity())
            {
                return;
            }

            // Vanilla only ever asks for a magnitude of 1. Anything less means it is either easing
            // into the final frame or obeying a thrust limit, so leave those alone.
            float magnitude = __result.magnitude;
            if (magnitude < 0.999f)
            {
                return;
            }
            float boost = Mathf.Min(1f / MaxAbs(__result), BetterAutopilot.ThrustCap);

            // Never remove more velocity in one step than is left, or vanilla's stop check overshoots.
            var rf = RefFrame(__instance);
            var body = Body(__instance);
            float maxThrust = Model(__instance).GetMaxTranslationalThrust();
            float remaining = (rf.GetVelocity() - body.GetVelocity()).magnitude;
            float perFrame = maxThrust * Time.fixedDeltaTime;
            boost = Mathf.Min(boost, remaining / Mathf.Max(perFrame, 1e-4f));
            if (boost > 1f)
            {
                __result *= boost;
            }
        }

        [HarmonyPostfix]
        [HarmonyPatch("ReadRotationalInput")]
        public static void ReadRotationalInput_Postfix(Autopilot __instance, ref Vector3 __result)
        {
            if (!BetterAutopilot.AutoOrient || BetterAutopilot.GameDefault || !IsShipAutopilot(__instance)
                || !__instance.IsFlyingToDestination() || __instance.IsMatchingVelocity())
            {
                return;
            }

            if (OWInput.IsInputMode(InputMode.ShipCockpit | InputMode.LandingCam)
                && Mathf.Abs(OWInput.GetValue(InputLibrary.pitch)) + Mathf.Abs(OWInput.GetValue(InputLibrary.yaw)) > 0.05f)
            {
                _playerRotateTime = Time.time;
            }
            if (Time.time < _playerRotateTime + PlayerOverrideSeconds)
            {
                return;
            }

            var model = Model(__instance);
            // Do not swing the ship about while scraping off a surface.
            if (_liftoffBody != null)
            {
                return;
            }
            if (!_keepTargetInView && (_alignDirWorld == Vector3.zero || _desiredThrustWorld.magnitude < 0.3f * model.GetMaxTranslationalThrust()))
            {
                return;
            }

            Transform transform = __instance.transform;
            if (_keepTargetInView)
            {
                _diagonal = Vector3.zero;
                __result += RotationInputToward(transform, model, Vector3.forward, _targetDirWorld, out _);
                return;
            }
            Vector3 desired = _alignDirWorld;
            _diagonal = PickDiagonal(transform.InverseTransformDirection(desired), _diagonal, _viewActive);
            Vector3 roll = _viewActive ? RollTowardTarget(transform, desired, _targetDirWorld) : Vector3.zero;
            __result += RotationInputToward(transform, model, _diagonal, desired, out _, roll);
        }

        /// <summary>
        /// Spinning about the thrust axis costs no thrust, so use that freedom to bring the target
        /// as close to the nose as the chosen diagonal allows.
        /// </summary>
        private static Vector3 RollTowardTarget(Transform transform, Vector3 thrustDir, Vector3 targetDir)
        {
            Vector3 nose = transform.forward;
            Vector3 nosePerp = nose - thrustDir * Vector3.Dot(nose, thrustDir);
            Vector3 targetPerp = targetDir - thrustDir * Vector3.Dot(targetDir, thrustDir);
            if (nosePerp.sqrMagnitude < 0.01f || targetPerp.sqrMagnitude < 0.01f)
            {
                return Vector3.zero;
            }
            nosePerp.Normalize();
            targetPerp.Normalize();
            float angle = Mathf.Atan2(Vector3.Dot(thrustDir, Vector3.Cross(nosePerp, targetPerp)), Vector3.Dot(nosePerp, targetPerp));
            return thrustDir * Mathf.Clamp(angle * 1.5f, -MaxTurnRate, MaxTurnRate);
        }

        /// <summary>
        /// Rotational input that turns the ship so <paramref name="localAxis"/> points along
        /// <paramref name="desiredWorld"/>. <paramref name="angle"/> is the remaining error in radians.
        /// </summary>
        public static Vector3 RotationInputToward(Transform transform, ThrusterModel model, Vector3 localAxis,
            Vector3 desiredWorld, out float angle, Vector3 extraSpin = default)
        {
            Vector3 current = transform.TransformDirection(localAxis);
            Vector3 axis = Vector3.Cross(current, desiredWorld);
            float sin = axis.magnitude;
            angle = Mathf.Atan2(sin, Vector3.Dot(current, desiredWorld));
            Vector3 targetSpin = sin > 1e-5f ? axis / sin * Mathf.Min(angle * 2f, MaxTurnRate) : Vector3.zero;
            targetSpin += extraSpin;

            Vector3 angularAccel = (targetSpin - model.GetAngularVelocity()) / 0.25f;
            Vector3 input = transform.InverseTransformDirection(angularAccel) / model.GetMaxRotationalThrust();
            input.x = Mathf.Clamp(input.x, -1f, 1f);
            input.y = Mathf.Clamp(input.y, -1f, 1f);
            input.z = Mathf.Clamp(input.z, -1f, 1f);
            return input;
        }

        /// <summary>Drops any manual-rotation pause and re-picks the diagonal (after a mode or toggle change).</summary>
        public static void ResumeOrientation()
        {
            _playerRotateTime = -100f;
            _diagonal = Vector3.zero;
        }

        private static Vector3 FlyStep(Autopilot autopilot)
        {
            var rf = RefFrame(autopilot);
            var body = Body(autopilot);
            var model = Model(autopilot);
            float maxThrust = model.GetMaxTranslationalThrust();

            Vector3 shipPos = body.GetWorldCenterOfMass();
            Vector3 targetPos = rf.GetPosition();
            float arrival = rf.GetAutopilotArrivalDistance();
            Vector3 toTarget = targetPos - shipPos;
            float distance = toTarget.magnitude;
            Vector3 relVel = body.GetVelocity() - rf.GetVelocity();

            if (!_liftoffDone && BetterAutopilot.AvoidObstacles)
            {
                Vector3? climb = LiftoffStep(autopilot, shipPos, maxThrust);
                if (climb.HasValue)
                {
                    return climb.Value;
                }
            }

            Vector3 aim = toTarget / Mathf.Max(distance, 0.001f);
            float pathLength = distance;
            bool detour = false;
            string blocker = null;
            if (BetterAutopilot.AvoidObstacles)
            {
                float planAccel = maxThrust * (_keepTargetInView ? 1f : BetterAutopilot.ThrustCap) * BetterAutopilot.BrakingMargin;
                detour = PredictivePlanner.Follow(shipPos, relVel, rf, arrival, planAccel, out aim, out pathLength, out blocker);
            }

            float remaining = pathLength - arrival;
            float closing = Vector3.Dot(relVel, aim);
            float straightClosing = Vector3.Dot(relVel, toTarget / Mathf.Max(distance, 0.001f));
            LogTelemetry(distance, arrival, remaining, relVel.magnitude, straightClosing, blocker, shipPos, body.GetVelocity());

            // Arrival always wins over obstacle avoidance, otherwise a detour can carry us into the target.
            // Hand off once inside the stop buffer, or the plan (which stops at the buffer) would hover there.
            if (distance - arrival <= StopBuffer + Mathf.Max(5f, straightClosing * 0.25f))
            {
                // Close enough: let vanilla null out the last bit of velocity and announce arrival.
                SetPhase(autopilot, "arrive", false, false);
                if (!_braking)
                {
                    FireRetroRockets(autopilot);
                }
                _desiredThrustWorld = Vector3.zero;
                autopilot.StartMatchVelocity(rf, ignoreThrustLimits: true);
                return Vector3.zero;
            }

            // Deceleration we can count on near the target, after its gravity pulls back on us.
            // With auto-align on, plan with the thrust we will have once aligned and count the time
            // still needed to turn as extra reaction lag; otherwise use what the current angle gives.
            float thrustFactor;
            float responseTime = ResponseTime;
            bool aligning = BetterAutopilot.AutoOrient && !_keepTargetInView && _diagonal != Vector3.zero && Time.time >= _playerRotateTime + PlayerOverrideSeconds;
            if (aligning)
            {
                thrustFactor = BetterAutopilot.ThrustCap;
                // Braking thrust is along the same line with the sign flipped, which costs no turning.
                float misalign = Mathf.Acos(Mathf.Clamp01(Mathf.Abs(Vector3.Dot(autopilot.transform.TransformDirection(_diagonal).normalized, aim))));
                responseTime += misalign / MaxTurnRate;
            }
            else
            {
                thrustFactor = Mathf.Min(BetterAutopilot.ThrustCap, 1f / MaxAbs(autopilot.transform.InverseTransformDirection(aim)));
            }
            float brakeAccel = maxThrust * thrustFactor * BetterAutopilot.BrakingMargin;
            var gravity = rf.GetOWRigidBody()?.GetAttachedGravityVolume();
            if (gravity != null)
            {
                brakeAccel -= gravity.CalculateGravityMagnitude(Mathf.Max(arrival, 1f));
            }
            brakeAccel = Mathf.Max(brakeAccel, 0.2f * maxThrust);

            // Fastest speed from which we can still stop in time, allowing for reaction lag. The plan
            // brakes at (nearly) full power; the safety comes from a fixed distance buffer instead.
            float lag = brakeAccel * responseTime;
            float stoppingRoom = Mathf.Max(0f, remaining - StopBuffer);
            float speed = Mathf.Sqrt(2f * brakeAccel * stoppingRoom + lag * lag) - lag;
            if (BetterAutopilot.MaxSpeed > 0f)
            {
                speed = Mathf.Min(speed, BetterAutopilot.MaxSpeed);
            }
            Vector3 desiredVel = aim * speed;

            // Relative acceleration = thrust + (our external forces - the target's acceleration).
            Vector3 externalRel = Forces(autopilot).GetForceAcceleration() - rf.GetAcceleration();
            Vector3 thrust = (desiredVel - relVel) / VelocityTimeConstant - externalRel;

            // Whatever the plan says, if our current momentum is carrying us into a body, get clear first.
            // Once triggered, keep evading for a moment so it does not flip-flop with the route plan.
            bool evading = false;
            if (BetterAutopilot.AvoidObstacles)
            {
                if (RoutePlanner.CheckCollision(shipPos, body.GetVelocity(), maxThrust * 0.8f, out Vector3 newEscape, out string newThreat))
                {
                    evading = true;
                    _escape = newEscape;
                    _threat = newThreat;
                    _evadeUntil = Time.time + EvadeHoldSeconds;
                }
                else if (Time.time < _evadeUntil)
                {
                    evading = true;
                }
            }
            _targetDirWorld = toTarget / Mathf.Max(distance, 0.001f);
            if (evading)
            {
                _viewActive = false;
                Vector3 escape = _escape;
                _alignDirWorld = escape;
                string threat = _threat;
                SetPhase(autopilot, $"evading {threat}", true, false);
                LogTelemetry(distance, arrival, remaining, relVel.magnitude, straightClosing, threat, shipPos, body.GetVelocity(), force: true);
                _desiredThrustWorld = escape * maxThrust * 10f;
                return FullThrust(autopilot.transform.InverseTransformDirection(escape));
            }
            _desiredThrustWorld = thrust;

            Vector3 lateral = relVel - aim * closing;
            if (!_braking && closing > speed + 1f && closing > 10f)
            {
                _braking = true;
                FireRetroRockets(autopilot);
            }
            // Slowing for arrival: braking proper, or thrust already pulling back near the braking curve.
            float along = Vector3.Dot(thrust, aim);
            if (_braking || (along < -0.2f * maxThrust && closing > 0.5f * speed && closing > 10f))
            {
                _decelerating = true;
            }
            _viewActive = _decelerating ? BetterAutopilot.ArrivalView : BetterAutopilot.DepartureView;

            // Align along the line of travel (thrust can run either way along it for free), unless a
            // big sideways correction is needed; that way a brief swing in the thrust direction while
            // switching from accelerating to braking does not turn the ship.
            Vector3 sideways = thrust - aim * along;
            _alignDirWorld = !_viewActive && sideways.magnitude > 0.5f * maxThrust * BetterAutopilot.ThrustCap
                ? thrust.normalized
                : aim;
            if (_braking)
            {
                SetPhase(autopilot, "braking", false, false);
            }
            else if (lateral.magnitude > Mathf.Max(5f, 0.2f * relVel.magnitude))
            {
                SetPhase(autopilot, detour ? $"lining up (around {blocker})" : "lining up", true, false);
            }
            else
            {
                SetPhase(autopilot, detour ? $"accelerating (around {blocker})" : "accelerating", false, true);
            }

            Vector3 input = autopilot.transform.InverseTransformDirection(thrust) / maxThrust;
            if (input.magnitude < 0.01f)
            {
                return Vector3.zero;
            }
            // Keep the direction, but let each axis run to its own limit.
            float largest = MaxAbs(input);
            if (largest > 1f)
            {
                input /= largest;
            }
            if (input.magnitude > BetterAutopilot.ThrustCap)
            {
                input = input.normalized * BetterAutopilot.ThrustCap;
            }
            return input;
        }

        /// <summary>
        /// Near a surface, climb straight away from the body first (holding still sideways relative to
        /// its turning surface) rather than skidding along the ground towards the route.
        /// Returns null once clear (or when not near a surface).
        /// </summary>
        private static Vector3? LiftoffStep(Autopilot autopilot, Vector3 shipPos, float maxThrust)
        {
            if (_liftoffBody == null)
            {
                _liftoffBody = RoutePlanner.LiftoffBody(shipPos);
                if (_liftoffBody == null)
                {
                    _liftoffDone = true;
                    return null;
                }
                BetterAutopilot.Log($"Lifting off from {_liftoffBody.Name}");
            }
            var surface = _liftoffBody.Body;
            Vector3 fromCenter = surface != null ? shipPos - surface.GetWorldCenterOfMass() : Vector3.zero;
            if (surface == null || fromCenter.magnitude > RoutePlanner.LiftoffClearRadius(_liftoffBody))
            {
                BetterAutopilot.Log($"Clear of {_liftoffBody.Name}, starting the route");
                _liftoffBody = null;
                _liftoffDone = true;
                return null;
            }

            Vector3 up = fromCenter.normalized;
            Vector3 relVel = Body(autopilot).GetVelocity() - surface.GetPointVelocity(shipPos);
            Vector3 externalRel = Forces(autopilot).GetForceAcceleration() - surface.GetAcceleration();
            Vector3 thrust = (up * LiftoffClimbSpeed - relVel) / VelocityTimeConstant - externalRel;
            SetPhase(autopilot, $"lifting off from {_liftoffBody.Name}", true, false);
            _desiredThrustWorld = thrust;
            _alignDirWorld = Vector3.zero;
            return FullThrust(autopilot.transform.InverseTransformDirection(thrust)) * Mathf.Clamp01(thrust.magnitude / (maxThrust * BetterAutopilot.ThrustCap));
        }

        private static Vector3 PickDiagonal(Vector3 localDir, Vector3 current, bool viewActive)
        {
            if (viewActive)
            {
                // The forward + up diagonal along the line to the target puts the target ahead and
                // above the nose, whether we are thrusting towards it or braking away from it.
                Vector3 view = BetterAutopilot.ThrusterAxes == 3
                    ? new Vector3(Sign(localDir.x), 1f, 1f).normalized
                    : new Vector3(0f, 1f, 1f).normalized;
                bool sameFamily = current != Vector3.zero && current.y > 0f && current.z > 0f
                    && (BetterAutopilot.ThrusterAxes == 2) == (current.x == 0f);
                if (sameFamily && Vector3.Dot(current, localDir) >= Vector3.Dot(view, localDir) - 0.05f)
                {
                    return current;
                }
                return view;
            }

            Vector3 best;
            if (BetterAutopilot.ThrusterAxes == 3)
            {
                best = new Vector3(Sign(localDir.x), Sign(localDir.y), Sign(localDir.z)).normalized;
            }
            else
            {
                // Two strongest axes.
                Vector3 abs = new Vector3(Mathf.Abs(localDir.x), Mathf.Abs(localDir.y), Mathf.Abs(localDir.z));
                int weakest = abs.x <= abs.y && abs.x <= abs.z ? 0 : abs.y <= abs.z ? 1 : 2;
                best = new Vector3(Sign(localDir.x), Sign(localDir.y), Sign(localDir.z));
                best[weakest] = 0f;
                best.Normalize();
            }

            // Hysteresis so it does not flip between diagonals when halfway.
            if (current != Vector3.zero && Vector3.Dot(current, localDir) >= Vector3.Dot(best, localDir) - 0.05f)
            {
                return current;
            }
            return best;
        }

        private static void SetPhase(Autopilot autopilot, string phase, bool liningUp, bool approaching)
        {
            LiningUp(autopilot) = liningUp;
            Approaching(autopilot) = approaching;
            if (phase != _lastPhase)
            {
                _lastPhase = phase;
                BetterAutopilot.Log($"Autopilot: {phase}");
            }
        }

        // Unit direction scaled so its largest axis is at full power (capped by the thruster setting).
        private static Vector3 FullThrust(Vector3 localDir)
        {
            Vector3 input = localDir / Mathf.Max(MaxAbs(localDir), 1e-4f);
            return input.magnitude > BetterAutopilot.ThrustCap ? input.normalized * BetterAutopilot.ThrustCap : input;
        }

        private static void LogTelemetry(float distance, float arrival, float remaining, float speed, float closing, string blocker,
            Vector3 shipPos, Vector3 shipVel, bool force = false)
        {
            // Every 2 s normally, every 0.5 s while evading.
            if (!BetterAutopilot.DebugLog || Time.time < _lastTelemetryTime + (force ? 0.5f : 2f))
            {
                return;
            }
            _lastTelemetryTime = Time.time;
            BetterAutopilot.Log($"  dist {distance:F0} (arrive at {arrival:F0}), path left {remaining:F0}, speed {speed:F0}, closing {closing:F0}"
                + (blocker != null ? $", avoiding {blocker}" : "") + RoutePlanner.DescribeNearest(shipPos, shipVel));
        }

        [HarmonyPrefix]
        [HarmonyPatch(nameof(Autopilot.Abort))]
        public static void Abort_Prefix(Autopilot __instance)
        {
            if (!__instance.enabled || !IsShipAutopilot(__instance))
            {
                return;
            }
            var resources = Resources(__instance);
            BetterAutopilot.Log($"Autopilot aborted (damaged {__instance.IsDamaged()}, thrusters usable {resources.AreThrustersUsable()}, fuel {resources.GetFuel():F0})"
                + $", called from {Diagnostics.Caller()}" + RoutePlanner.DescribeNearest(Body(__instance).GetWorldCenterOfMass(), Body(__instance).GetVelocity()));
        }

        [HarmonyPrefix]
        [HarmonyPatch(nameof(Autopilot.SetDamaged))]
        public static void SetDamaged_Prefix(Autopilot __instance, bool damaged)
        {
            if (damaged && IsShipAutopilot(__instance))
            {
                BetterAutopilot.Log("Autopilot component damaged");
            }
        }

        private static void FireRetroRockets(Autopilot autopilot)
        {
            var handler = AccessTools.Field(typeof(Autopilot), "OnFireRetroRockets").GetValue(autopilot) as Autopilot.FireRetroRocketsEvent;
            handler?.Invoke();
        }

        private static float MaxAbs(Vector3 v) => Mathf.Max(Mathf.Abs(v.x), Mathf.Max(Mathf.Abs(v.y), Mathf.Abs(v.z)));

        private static float Sign(float value) => value < 0f ? -1f : 1f;
    }
}
