---
title: Combat Robot Drive and Weapon Motor Simulation
description: 'Set motor KV, current, battery voltage, gear ratio, and weapon or wheel drive settings in the KineticArena combat robot simulator.'
---

# Combat robot drive and weapon motors

Motor settings are stored with each bot setup. Configure motors in **Bot
workshop → Drive motor settings** or **Weapon motor settings**.

## Shared drive motors

Assign wheel assemblies to motor groups. Put wheels driven by the same physical
motor in the same group. Set the number of motors in that group and the peak
current for each motor. The group's available torque is shared across its
assigned wheel assemblies. **Couple wheels in this motor group** constrains
their shaft speeds using their configured reductions and wheel-axis directions.
This is a rigid linkage approximation: it does not model chain stretch, backlash,
slip, or differential action. Turn it off for independently driven outputs.

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
zero. Without resistance, KineticArena uses an approximate current-limited torque
curve. Peak current is treated as torque-producing motor current. Optional pack
internal resistance adds a first-order voltage sag estimate from estimated
drive current. This estimate does not model cell chemistry, state of charge,
voltage recovery, wiring, winding heating, or thermal derating. Estimated
current is approximate, so use measured pack data for calibration.

Each imported part can be excluded from collision while remaining visible and
mass-bearing. MuJoCo uses a convex hull for each imported collision body, so
holes and deep concave cuts are filled. A weapon can use several
`CollisionProxy_` helper bodies to approximate its shape with multiple convex
pieces; see [Fusion import](fusion-import.md#weapon-collision-proxies).
**Show STL collision shape** displays the collision hulls in the CAD preview.

## What the motor model changes

With torque mode enabled, throttle commands motor torque and loaded wheel RPM
emerges from the simulated load. Full throttle's no-load speed estimate depends
on the speed rating, pack voltage, and gear ratio. If a side has no motor data,
the current build uses a legacy 424 RPM target and reports that motor data is
missing.

The optional manual wheel RPM override uses a speed servo and bypasses the
motor model. Use it for a forced-speed check, not as a realistic motor setup.

Assemblies have an optional **Impact failure threshold (N)**. Zero (the default)
keeps failures disabled. If a contact on that assembly exceeds its configured
normal-force threshold, KineticArena disables the assembly's drive actuator for the
rest of that run. This is a simple configurable trip condition; it does not
fracture, detach, deform, or change the assembly's collision geometry. Restart
or rebuild the simulation to clear the failed state.

## Weapon motor

The weapon torque model uses motor speed/current data and the weapon assembly's
inertia about its configured axis. Holding the weapon control applies motor
torque; releasing it lets the weapon coast under configured bearing and
aerodynamic drag, when those optional values are entered. The UI also estimates
time to reach 90% of no-load speed. The estimate omits battery sag, impact
loading, changing motor temperature, and heat effects.

Printed-part mass is an area-based shell and infill estimate. It does not
simulate slicer toolpaths exactly, print strength, anisotropy, or flex. The CAD
collision surface remains the imported mesh approximation.

When the weapon torque model is off, the legacy target-RPM and spin-up controls
remain available. The stock practice model's weapon target is 30,000 RPM.
