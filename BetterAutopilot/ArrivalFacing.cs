using UnityEngine;

namespace BetterAutopilot
{
    /// <summary>
    /// After the autopilot arrives, turns the ship's nose to the destination with the rotation
    /// thrusters (no translation). Stops once aligned, or as soon as the player steers, restarts
    /// the autopilot or leaves the seat.
    /// </summary>
    public static class ArrivalFacing
    {
        private const float AlignedAngle = 1f * Mathf.Deg2Rad;
        private const float GiveUpSeconds = 15f;

        private static Autopilot _subscribed;
        private static ReferenceFrame _target;
        private static float _startTime;

        /// <summary>Called whenever the ship autopilot starts a flight.</summary>
        public static void Watch(Autopilot autopilot, ReferenceFrame target)
        {
            _target = null;
            if (_subscribed != autopilot)
            {
                // A new ship every loop, so (re)subscribe to whichever autopilot is flying now.
                autopilot.OnArriveAtDestination += OnArrive;
                _subscribed = autopilot;
            }
            _pendingTarget = target;
        }

        private static ReferenceFrame _pendingTarget;

        private static void OnArrive(float arrivalError)
        {
            if (!BetterAutopilot.FaceTargetOnArrival || BetterAutopilot.GameDefault || _pendingTarget == null)
            {
                return;
            }
            _target = _pendingTarget;
            _startTime = Time.time;
            BetterAutopilot.Log("Arrived: turning to face the target");
        }

        public static void FixedUpdate()
        {
            if (_target == null)
            {
                return;
            }

            var ship = Locator.GetShipBody();
            var model = ship?.GetComponent<ThrusterModel>();
            var autopilot = ship?.GetComponent<Autopilot>();
            bool playerSteering = Mathf.Abs(OWInput.GetValue(InputLibrary.pitch)) + Mathf.Abs(OWInput.GetValue(InputLibrary.yaw)) > 0.05f;
            if (ship == null || model == null || _target.GetOWRigidBody() == null || !PlayerState.AtFlightConsole()
                || (autopilot != null && autopilot.enabled) || playerSteering || Time.time > _startTime + GiveUpSeconds)
            {
                _target = null;
                return;
            }

            Vector3 toTarget = (_target.GetPosition() - ship.GetWorldCenterOfMass()).normalized;
            Vector3 input = AutopilotPatches.RotationInputToward(ship.transform, model, Vector3.forward, toTarget, out float angle);
            if (angle < AlignedAngle && model.GetAngularVelocity().magnitude < 0.02f)
            {
                _target = null;
                return;
            }
            model.AddRotationalInput(input);
        }
    }
}
