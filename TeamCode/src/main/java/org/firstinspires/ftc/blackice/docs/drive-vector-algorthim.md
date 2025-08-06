---
title: Drive Vector Algorithm
description: How Pedro Path Calculates the Drive Vector
---
// (this is the wrong category i think) //
import PedroImplementation from "@/app/PedroImplementation";

Note: This algorithm runs twice, once for the robot’s forward axis and once for the lateral axis, and then combines the results into a vector. This allows the robot to handle different decelerations per axis (e.g. mecanum wheels decelerate faster laterally than forward). The kinematics equations are also slightly modified to account for signed distance.

### 1. Target Velocity Calculation

First, it calculates the target velocity the robot should have to decelerate at a specified rate and stop exactly at the end of the path. This is done using the kinematics formula:

```java
targetVelocity = vᵢ = sqrt(-2·a·d)
```

Where:
- `vᵢ` is our target velocity
- `a` is the desired deceleration (should be negative)
- `d` is the distance remaining (in code, modified for signed distance).

### 2. Error Control and Feedforward

To control velocity, a <u>PIDF controller</u> is used. The most important terms are:

- **Proportional (P)** - corrects based on error: `error * kP`
- **Feedforward (F)** - prediction for the required velocity to reach the target velocity directly: `targetVelocity * kF`
  For example: if you want to go `60 in/s`, and `kF = 0.015`, the result is `0.9 power`; for `30 in/s`, it’s `0.45 power`.

This works well when cruising at a constant velocity. But during deceleration, momentum becomes a problem. The feedforward doesn’t account for the robot’s momentum and inertia, so it might still apply power when it should be reducing power much more aggressively. The proportional term alone can't fix this because it’s reactive, not predictive.

### 3. Accounting for Momentum (Zero Power Decay)

To fix this, Pedro Path uses the concept of zero power acceleration, the deceleration the robot naturally experiences when power is cut (basically, its momentum).

Using this value, it estimates the velocity the robot would naturally have at the end of the path if it were to coast with **no power**. That’s calculated using:
```
velocityAtEndOfPath = sqrt(vᵢ² + 2·a·d)
```

Where:
- `velocityAtEndOfPath` is the velocity the robot would be at the end of the path if it stopped applying power to the drivetrain and just had its momentum,
- `vᵢ` is the current velocity,
- `a` is the zero power deceleration,
- `d` is the distance remaining.

Then it calculates a value called `zeroPowerDecay` which tells us how much velocity velocity that needs to be lost due to momentum.
```
zeroPowerDecay = currentVelocity - velocityAtEndOfPath
```

Finally, it adjusts the target velocity by subtracting the `zeroPowerDecay` or accounting for the amount of velocity that needs to be lost due to momentum.

```
targetVelocity = targetVelocity - zeroPowerDecay
```
This lets the robot reduce its feedforward power appropriately, so it slows down predictively, not just reactively.
