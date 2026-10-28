# Drive and weapon motors

Motor settings are stored with each bot setup. Configure motors in **Bot
workshop → Drive motor settings** or **Weapon motor settings**.

## Shared drive motors

Assign wheel assemblies to motor groups. Put wheels driven by the same physical
motor in the same group. Set the number of motors in that group and the peak
current for each motor. The group's available torque is shared across its
assigned wheel assemblies; the current model does not lock wheel speeds
together through a chain, belt, or differential.

The **Drive layout** can be **2WD** or **4WD**. For 2WD, select the front or
rear axle. The other axle's assignments and settings remain saved and return
when you switch back to 4WD.

## Motor data

For each motor group, set:

1. Motor type: **Brushless (BLDC)** or **Brushed DC**.
2. A speed rating, entered as **Kv (RPM/V)** or **RPM at voltage**.
3. The test/rated voltage when entering RPM at voltage.
4. Peak motor current and the robot's battery pack voltage.
5. Motor count and gear ratio.

Enter a no-load RPM rating when the manufacturer provides one. The simulator
scales that rating with pack voltage. Gear ratio is entered as input:output,
for example `16:1`; the simulator divides motor speed by the reduction ratio.

Phase-to-phase resistance for brushless motors, armature resistance for
brushed motors, and an ESC current limit are optional. Leave unknown values at
zero. Without resistance, CombatSim uses an approximate current-limited torque
curve. Peak current is treated as torque-producing motor current. The model
does not include battery sag, winding heating, or thermal derating, so peak
torque can persist indefinitely in the simulation.

## What the motor model changes

With torque mode enabled, throttle commands motor torque and loaded wheel RPM
emerges from the simulated load. Full throttle's no-load speed estimate depends
on the speed rating, pack voltage, and gear ratio. If a side has no motor data,
the current build uses a legacy 424 RPM target and reports that motor data is
missing.

The optional manual wheel RPM override uses a speed servo and bypasses the
motor model. Use it for a forced-speed check, not as a realistic motor setup.

## Weapon motor

The weapon torque model uses motor speed/current data and the weapon assembly's
inertia about its configured axis. Holding the weapon control applies motor
torque; releasing it lets the weapon coast under modeled losses. The UI also
estimates time to reach 90% of no-load speed. The estimate omits battery sag,
impact loading, and heat effects.

When the weapon torque model is off, the legacy target-RPM and spin-up controls
remain available. The stock practice model's weapon target is 30,000 RPM.
