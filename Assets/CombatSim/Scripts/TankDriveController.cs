using UnityEngine;

namespace CombatSim
{
    [RequireComponent(typeof(Rigidbody))]
    public sealed class TankDriveController : MonoBehaviour
    {
        [Min(1f)]
        public float driveWheelSpeedRpm = 424.4f;

        [Min(1f)]
        public float wheelRadiusMm = 45f;

        [Min(1f)]
        public float maximumDriveForcePerSideNewtons = 180f;

        [Min(0f)]
        public float tractionCoefficient = 1.1f;

        [Min(50f)]
        public float trackSeparationMm = 280f;

        [Min(50f)]
        public float contactRayLengthMm = 200f;

        private readonly RaycastHit[] groundHitBuffer = new RaycastHit[8];
        private Rigidbody body;

        private void Awake()
        {
            body = GetComponent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            float throttle = (Input.GetKey(KeyCode.W) ? 1f : 0f)
                - (Input.GetKey(KeyCode.S) ? 1f : 0f);
            float steering = (Input.GetKey(KeyCode.D) ? 1f : 0f)
                - (Input.GetKey(KeyCode.A) ? 1f : 0f);

            ApplyDriveInput(throttle, steering);
        }

        public void ApplyDriveInput(float throttle, float steering)
        {
            throttle = Mathf.Clamp(throttle, -1f, 1f);
            steering = Mathf.Clamp(steering, -1f, 1f);

            float leftInput = Mathf.Clamp(throttle - steering, -1f, 1f);
            float rightInput = Mathf.Clamp(throttle + steering, -1f, 1f);

            ApplyTrackForce(-1f, leftInput);
            ApplyTrackForce(1f, rightInput);
        }

        private void ApplyTrackForce(float side, float input)
        {
            float halfTrackSeparation = Units.MillimetresToMetres(trackSeparationMm) / 2f;
            float rayLength = Units.MillimetresToMetres(contactRayLengthMm);

            Vector3 localContact = new Vector3(side * halfTrackSeparation, 0f, 0f);
            Vector3 rayOrigin = transform.TransformPoint(localContact)
                + Vector3.up * (rayLength * 0.75f);
            float castDistance = rayLength * 1.75f;
            int hitCount = Physics.RaycastNonAlloc(
                rayOrigin,
                Vector3.down,
                groundHitBuffer,
                castDistance);

            RaycastHit groundHit = default;
            float nearestGroundDistance = float.MaxValue;
            bool foundGround = false;
            for (int i = 0; i < hitCount; i++)
            {
                RaycastHit candidate = groundHitBuffer[i];
                if (candidate.collider == null
                    || candidate.rigidbody == body
                    || candidate.distance >= nearestGroundDistance)
                {
                    continue;
                }

                groundHit = candidate;
                nearestGroundDistance = candidate.distance;
                foundGround = true;
            }

            if (!foundGround)
            {
                return;
            }

            Vector3 forward = Vector3.ProjectOnPlane(transform.forward, groundHit.normal).normalized;
            if (forward.sqrMagnitude < 0.001f)
            {
                return;
            }

            Vector3 sideDirection = Vector3.ProjectOnPlane(transform.right, groundHit.normal).normalized;
            Vector3 forcePoint = new Vector3(
                groundHit.point.x,
                body.worldCenterOfMass.y,
                groundHit.point.z);
            Vector3 pointVelocity = body.GetPointVelocity(forcePoint);
            float forwardSpeed = Vector3.Dot(pointVelocity, forward);
            float wheelRadiusMetres = Units.MillimetresToMetres(wheelRadiusMm);
            float targetAngularSpeed = input * driveWheelSpeedRpm * (2f * Mathf.PI / 60f);
            float targetSpeed = targetAngularSpeed * wheelRadiusMetres;
            float desiredForce = (targetSpeed - forwardSpeed) * 120f;

            float normalLoad = body.mass * Physics.gravity.magnitude / 2f;
            float tractionLimit = tractionCoefficient * normalLoad;
            float driveForce = Mathf.Clamp(
                desiredForce,
                -Mathf.Min(maximumDriveForcePerSideNewtons, tractionLimit),
                Mathf.Min(maximumDriveForcePerSideNewtons, tractionLimit));

            float lateralSpeed = Vector3.Dot(pointVelocity, sideDirection);
            float lateralForce = Mathf.Clamp(
                -lateralSpeed * 60f,
                -tractionLimit,
                tractionLimit);

            Vector3 force = forward * driveForce + sideDirection * lateralForce;
            body.AddForceAtPosition(force, forcePoint);
        }
    }
}
