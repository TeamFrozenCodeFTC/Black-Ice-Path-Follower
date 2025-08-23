---
layout: on-this-page
title: "Quadratic-Damped PID"
nav_order: 3
---

# Our Key Innovation, the Quadratic-Damped PID

Our key innovation about our follower is our quadratic-damped PID. We include this quadratic damping term in our translational PID, allowing it to be nearly **5x faster** at decelerating, more aggressive, and accurate while minimizing premature deceleration for faster deceleration. 
You can learn about how we came up with this idea with [our first empirical approach](https://github.com/TeamFrozenCodeFTC/Black-Ice-Path-Follower/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/blackice/docs/path-follower-evolution.md#v30---corrective-braking-using-a-quadratic-damped-pid)

Pseudo code of PD controller with quadratic damping
```
error = target - current
derivative = kD * -velocity
quadraticDamping = kQ * -velocity * abs(velocity)
proportional = error * proportionalConstant
outputPower = proportional + derivative + quadraticDamping
```

## Why Quadratic Damping Is Needed?
We wanted to brake as fast as [zero power brake mode](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.ZeroPowerBehavior.html#BRAKE), so that required quadratic regression to model the realistic, non-linear braking behavior of the robot with a quadratic-damped term. We later realized that applying small amounts of reverse power to the wheels uses the same system as zero power brake mode, but allows us to correct and control the robot's position. If you use a less aggressive PID controller that simply coasts to the target, quadratic damping isn’t necessary. It’s only required when you want high speed and braking that requires replicating the robot’s real braking dynamics at high speeds.

### Why is the Braking Non-Linear?
When you reverse the motor power, the motor’s internal back-EMF also reverses. This causes the motor to brake rapidly using its own momentum, even with very small power inputs like -0.00001. Essentially, the faster the wheel was initially spinning, the stronger the braking force generated. This is exactly how the zero power brake mode brakes.

In an ideal scenario, the braking force would be directly proportional to velocity. However, when the robot’s powered wheels are braking to a stop, the deceleration is not perfectly linear. The friction involved in braking causes the relationship between velocity and braking distance to become **non-linear**. This is why we call `kD`: `kBrake` and `kQ`: `kFriction` in our code.

## How much difference does the Quadratic Damping make?
Other libraries, such as Pedro Path and Roadrunner, rely on less aggressive, slower PID controllers that rely more on coasting to the target position. This is a more predictable constant deceleration of around `-40in/s` or `45in` of distance to stop while going `60 in/s`. However, if you use a more aggressive PID controller with quadratic-damping that makes use of back-EMF, you can get much faster deceleration of around `10 in` of distance to stop while going `60in/s`. This is nearly **5 times faster deceleration**, plus the extra time you have to accelerate instead of braking.

### Pros
- **Faster** – spends less time decelerating, more time accelerating
- **More Accurate** – higher proportional constant reduces steady-state error
- **More Responsive** – reacts quickly without oscillation due to quadratic damping
### Trade-offs
- **Higher Power Usage** – requires more voltage, applying reverse power to brake faster
- **Less Smooth Stops** – quick braking can cause slight shaking when first coming to a stop


# How does -velocity = Derivative term in PID?
Let the positional error be:  

$error = target_{pos} - current_{pos}$

Taking the derivative of both sides:  

$\frac{d}{dt}(error) = \frac{d}{dt}(target_{pos}) - \frac{d}{dt}(current_{pos})$

If the target position is constant, then:  

$\frac{d}{dt}(\text{target}_{\text{pos}}) = 0$

Then:  

$\frac{d}{dt}(error) = -\frac{d}{dt}(current_{pos}) = -velocity$

Therefore, when the target position is fixed, the derivative of the position error is simply the **negative of the current velocity**.

