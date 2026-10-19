using Mujoco;
using UnityEngine;

namespace CombatSim
{
    // Attach to the imported MJCF root. MuJoCo owns every moving body.
    public sealed class MujocoRobotController : MonoBehaviour
    {
        [Min(0f)] public float wheelSpeedRpm = 424f;
        [Min(0f)] public float weaponSpeedRpm = 30000f;
        [Min(0f)] public float weaponSpinUpSeconds = 1.5f;
        [Tooltip("Enable left-stick driving and a gamepad weapon button.")]
        public bool enableUsbGamepad = true;
        [Range(0f, 0.5f)] public float gamepadDeadzone = 0.15f;

        private MjActuator frontLeft;
        private MjActuator rearLeft;
        private MjActuator frontRight;
        private MjActuator rearRight;
        private MjActuator weapon;
        private float weaponCommand;

        private void Start()
        {
            frontLeft = FindActuator("front_left_drive");
            rearLeft = FindActuator("rear_left_drive");
            frontRight = FindActuator("front_right_drive");
            rearRight = FindActuator("rear_right_drive");
            weapon = FindActuator("weapon_drive");
        }

        private static MjActuator FindActuator(string actuatorName)
        {
            foreach (MjActuator actuator in FindObjectsByType<MjActuator>())
            {
                if (actuator.name == actuatorName)
                {
                    return actuator;
                }
            }

            Debug.LogError("Missing MuJoCo actuator: " + actuatorName);
            return null;
        }

        private void FixedUpdate()
        {
            float throttle = (Input.GetKey(KeyCode.W) ? 1f : 0f)
                - (Input.GetKey(KeyCode.S) ? 1f : 0f);
            float turn = (Input.GetKey(KeyCode.A) ? 1f : 0f)
                - (Input.GetKey(KeyCode.D) ? 1f : 0f);

            if (enableUsbGamepad && Input.GetJoystickNames().Length > 0)
            {
                float stickThrottle = Input.GetAxisRaw("Vertical");
                float stickTurn = -Input.GetAxisRaw("Horizontal");
                if (Mathf.Abs(stickThrottle) > gamepadDeadzone) throttle = stickThrottle;
                if (Mathf.Abs(stickTurn) > gamepadDeadzone) turn = stickTurn;
            }
            float wheelSpeedRadiansPerSecond = wheelSpeedRpm * (2f * Mathf.PI / 60f);
            float leftSpeed = throttle * wheelSpeedRadiansPerSecond;
            float rightSpeed = throttle * wheelSpeedRadiansPerSecond;

            if (throttle == 0f)
            {
                // Tank turn: opposite wheel directions rotate in place.
                float pivotSpeed = wheelSpeedRadiansPerSecond;
                leftSpeed = -turn * pivotSpeed;
                rightSpeed = turn * pivotSpeed;
            }
            else if (turn != 0f)
            {
                // Slow the inside side while moving. Reverse swaps which side
                // is inside so A and D keep the same turn directions with S.
                if (turn * throttle > 0f) leftSpeed = 0f;
                else rightSpeed = 0f;
            }

            float weaponSpeedRadiansPerSecond = weaponSpeedRpm * (2f * Mathf.PI / 60f);
            bool gamepadWeapon = enableUsbGamepad &&
                (Input.GetKey(KeyCode.JoystickButton4) || Input.GetKey(KeyCode.JoystickButton0));
            float target = (Input.GetKey(KeyCode.Space) || gamepadWeapon)
                ? weaponSpeedRadiansPerSecond
                : 0f;
            float step = weaponSpeedRadiansPerSecond * Time.fixedDeltaTime
                / Mathf.Max(0.01f, weaponSpinUpSeconds);
            weaponCommand = Mathf.MoveTowards(weaponCommand, target, step);

            if (frontLeft != null) frontLeft.Control = leftSpeed;
            if (rearLeft != null) rearLeft.Control = leftSpeed;
            if (frontRight != null) frontRight.Control = rightSpeed;
            if (rearRight != null) rearRight.Control = rightSpeed;
            // Negative Y rotation lifts the frontmost drum tooth into a hit.
            if (weapon != null) weapon.Control = -weaponCommand;
        }
    }
}
