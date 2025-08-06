Black Ice is a **Reactive and Predictive Path Follower** developed by __FTC Team #18535__, Frozen Code.
It is designed to provide more high-speed, efficient, and effective path following by using a specialized positional error controller that **predicts braking displacement.**.

Requirements:

- **Odometry Wheels** or some form of localization that can determine the robot's position and heading.
- Currently requires **mecanum wheels** but there is support for other drivetrains if you implement the Drivetrain interface.

## The History and Evolution of Our Custom Path Follower, Black Ice

### Overview
- v1.0 – Wheel encoder + IMU
  - v1.1 - Field-centric teleOp
- v2.0 – Odometry-based brute-force movement
- v3.0 – PredictiveBrakingController
- v4.0 – Dynamic Lookahead Follower
- v5.0 – Sophisticated Path Follower
  - translational, drive, heading, and deceleration profiles

The beginnings of Black ice started in our first season, INTO THE DEEP 2024-2025, with the development of our first autonomous frameworks. Our small framework was later was fully developed into a sophisticated follower in the 2025 off-season.

### v1.0 (2024) – Wheel Encoder, IMU, Zero Power Brake Mode Stop, and Field-Centric TeleOp

Used in our first ever competition in December. In Auto, Combined a single wheel encoder to estimate linear displacement with an IMU for heading lock. In teleOp it used the IMU to transform a target field-relative vector into a robot-relative vector.

Once the robot reached the target position, it would stop by setting the power to zero on zero power brake mode. We were limited to about 0.5 power since any faster would make the robot overshoot the target and could make the encoder wheel slip. Movement was basic but multi-directional with our mecanum wheels. Lacked speed and accuracy and could not follow paths.

#### Field-Centric TeleOp

After we realized we could retrieve the robot's heading from the IMU, it sparked the idea of reversing the heading to transform controller input into field-centric movement. This simple concept became a foundational step in developing vector following, which is essential for path following.

**Fun Fact:** At the time we had no idea that it was even called field-centric teleOp. We used to call it "controller relative teleOp" because it was relative to the controller's orientation if the controller was relative to the field.

### v2.0 - Odometry Wheels + Zero Power Brake Mode Stop

// Why didn't we switch to roadrunner or pedro path here?
Why we developed our own?
- We were interested in path following and we wanted to further the field
- To intuitively learn, hands-on, how path following works in robotics
- Potentially find faster or more efficient solutions
- Allow for more customization

Used goBILDA pinpoint odometry to move toward a target and then relied on zero power brake mode to stop.

Using separate non-powered wheels to track distance allowed for much more speed because we could apply braking force opposite to the robot's motion and didn't have to worry about if the powered wheels slightly slipped.

So far, this prototype could only go from point to point; it could not path follow with lines or curves.

#### Preventing Overshoot

This obviously would instantly overshoot the target position because it would only brake after it reached the target position. To fix this, we decided to run the robot at different speeds, turn on zero power brake mode, and calculate the distance the robot took to brake. With those data points we found that it was close to the form of `ax^2 + bx` so we used quadratic regression in that form to derive a function that accurately predicts the required braking distance at any speed.

We commonly found our coefficients to be around `a=0.001` and `b=0.07`.

- The `a` term accounts for friction from the robot's momentum from slipping. This makes it much more accurate at high velocities when friction is more dominant than the wheels' braking force.
- the `b` term is basically the robot's braking force from the wheels. Later on we would realize that just this term in our future predictive positional controller was just a more empirical version of the Derivative term. But the `a` quadratic term allows for a much more accurate and aggressive positional controller because it is more closely related to how the robot actually brakes. A simple PID positional controller can work if you tune the deceleration to be the robot's naturally deceleration but with more aggressive PID controllers it began to too harshly correct at lower velocities and overshoot at higher velocities.

In this prototype version, it would just turn on zero power brake mode if the distance was greater than the distance remaining to the target point. This worked okay, but it could not correct while it was braking. In next prototype version we would fix this issue by turning it into a simple proportional controller.


// vector braking displacement, seperate for lateral and forward

### v3.0 (Last version of 2024-2025 season) - PredictiveBrakingController Predictive back-EMF Braking

Used in our second competition in January 2025, and was the first time we called it Black-Ice.

In the previous prototype version, it would just turn on zero power brake mode if the braking distance was greater than the distance remaining to the target point. This worked okay, but it could not correct while it was braking. In this prototype version we fixed this issue by turning it into a simple proportional controller. 

Pseudo code implementation of the predictive braking controller:
```java
// Note: all of these variables are vector quantities with (x,y)
predictedBrakingDisplacement = a*velocity*abs(velocity) + b*velocity
predictedPositionAfterBraking = current + predictedBrakingDisplacement
error = target - predictedPositionAfterBraking
power = error * proportionalConstant
```

TODO go stop at the position or go past it without overshooting into the next target

We used a proportional constant of `0.5` *actually * 1 at first but then changed to 0.5 still don't know why we may have been wrongly scaled before

**Fun Fact:** We originally thought our algorithm was falling apart here because it would be quite a few inches off from the target, so we tried adding Integral terms but eventually we figured out that one of our odometry pods was just defective.

### v4.0 (Beginning of 2025 off-season) – First Path Follower
Made follower using a lookahead based off the predicted braking displacement. Followed based of a bunch of points spread along the path by the robot's 

### v5.0 (2025 off-season) - 
Started to implement a sophisticated follower that could follow more than just points, could follow continuous paths such as Bezier curves and lines. Used translational, full power drive vector, and 

todo back-emf, lower power braking, etc Predictive EMF Braking

Modeled braking displacement with kP/kQuad terms and actively applied reverse power based on back-EMF to both slow and correct position error.



//Made separate braking displacement models for forward vs lateral movement with mecanum wheels. but after testing it works to just use the same model for both axis since it is the equivalent of halfing different PIDs for each axis which is a bit unnecessary and just requires more tuning. There is support for this tho.


Things to note: power reverse of back-EMF
Modeled braking distance with kP/kQuad terms and actively applied reverse power based on back-EMF to both slow and correct position error.

// FIXME
Things to note after testing: our model basically simulated the zero power brake mode but with correction. the zero power brake mode uses back-EMF to make the wheels stop spinning and a fact is that if you just reverse the direction of the wheels for example form +1 to -0.001 it will basically behave like zero power brake mode until it gets to lower velocities where -0.001 doesn't do much and where a power like -0.3 would slow down more at lower velocities.

## Our contributions to Pedro Path
- Told them about the predictive positional controller

```
PathRoutine path = follower.pathRoutineBuilder()
    .curveTo(new Pose(24, 12), new Pose(48, 0))
        .withConstantHeading(90)
    .lineTo(new Pose(48, 24))
    .stop()
    .build()
```
high-speed
More empirical

#### Why is it called Black Ice?

We named our path follower Black Ice because it captures the way our robot moves—effortlessly, sliding to the exact position like it's gliding over a sheet of black ice. It doesn’t just handle smooth paths, it can navigate sharp turns and tight spaces, as if it were made to drive on ice.

## Our Predictive Positional Error Controller

First thing you need to understand about ftc motors is EMF braking and that reversing the motion of the wheels at high speeds does.
Unlike traditional path-following libraries that use smooth but weak positional PIDs, blackIce uses the fact
of a quadratic term to account for the robot's momentum and friction of the wheels. You can think of D as just predicting the braking distance for a model that gives force proportional to the velocity
. `error * kP - velocity * kD = (error - velocity * kD/kP) * kP` this just makes kD rely on kP which is actually beneficial since kP and kD are usually scaled together.

Unlike traditional path-following libraries that gradually slow the robot down,
Black Ice dynamically calculates the optimal braking distance based on the robot’s current momentum.
This allows the robot to maintain full power for as long as possible, only beginning to brake at the precise moment needed.
By predicting the robot's position in real time, it can also navigate curved paths because it predicts error before it happens.
![img_2.png](img_2.png)
convert

Black Ice is a **Reactive and Predictive Path Follower** developed by __FTC Team #18535__, Frozen Code.
It is designed for simple, efficient, effective and high-speed path following with minimum tuning.
Black Ice predicts error before it even happens by **predicting real-time, directional braking distance**.
Black Ice has the ability to maintain full power for as long as possible by dynamically calculating the optimal braking distance based on the robot’s current speed. If you need to slow down, Black Ice also supports velocity and acceleration constraints with PIDF tuning.

By predicting error, it allows the robot to go at full power and minimize braking. This allows it to go faster and brake minimally.

Predicting the robot's positon based off the braking distance, it allows the robot to go full power.
Aims to predict error before it even happens by predicting real-time, directional braking distance.
Black Ice has the ability to maintain full power for as long as possible by dynamically calculating the optimal braking distance based on the robot’s current speed. With more tuning Black Ice also supports velocity and acceleration constraints.

Designed for simple, efficient, effective and high-speed path following with minimum tuning
(but also supports velocity and acceleration constraints with more tuning).
When wanting full speed, Black Ice dynamically predicts the optimal braking distance based on the robot's current speed.
This allows the robot to maintain full power for as long as possible, only beginning to brake at the precise moment needed. By predicting the robot's position in real time, it can also navigate curved paths because it predicts error before it even happens.

turning uses a simple Proportional controller, (can PIDF work for heading too?)
+support for velocity and acceleration constraints using PIDF controller doesn't have to use full power
Predictive braking feedforward

More Predictive than Reactive – Instead of reacting to sudden changes, Black Ice anticipates braking needs.

- ✅ Eliminates Overshoot – Prevents aggressive corrections that can cause oscillations.
- ✅ Adapts to Different Speeds – Braking distance scales with velocity instead of using a fixed constant.

🛠 Potential Enhancements
Use a Logistic (Sigmoid) Braking Curve:
Instead of a linear braking factor, use a smooth nonlinear transition:

brakingFactor
=============

1
1
+
𝑒
−
𝑘
(
positionError
−
𝑑
stop
)
brakingFactor=
1+e
−k(positionError−d
stop

)

1

This avoids abrupt power drops near the target.

Adaptive Regression Updates:
Continuously log actual braking distances and adjust a, b, c dynamically.

Velocity Error Integration:
Add a velocity error term:

𝑘
𝐷
×
(
targetVelocity
−
currentVelocity
)
kD×(targetVelocity−currentVelocity)
for finer control.

and high-speed

Unique, key features:

- Zero Manual, Arbitrary Tuning
- Modular Customization
- Bezier Curves

Black Ice is tailored for teams with odometry wheels looking for simple, high-speed path execution with the option of modular customization.

# Usage

### Requirements: Odometry Wheels / Pinpoint Odometry Processor with omni-directional wheels (meacum wheels)

Black Ice intentionally uses the the slippage of the wheels in order to stop faster. This requires separate dead wheels in order to not get off.

## Initializing at start of OpMode

```java
Follower.init(this);
```
## Point-to-Point Movements

All of these methods are blocking, meaning they wait until the robot has reached its position.
If you want non-blocking methods or want to incorporate hardware during movements see [link](#More-Advanced-Modular-Movements).
Moving the robot to a point and stop:

```java
// Moves the robot 24 inches in the y direction at a heading of 90 degrees and stops.
Movement myMovement = MovementBuilder.stopAtPosition(0, 24, 90).build();
myMovement.waitForMovement();
// OR
myMovement.start();
while (!myMovement.isFinished()) {
    myMovement.update();
}
```
![img.png](img.png)

Moving the robot through a point:

```java
// Moves the robot 12 inches in x direction and 24 inches in the y direction at a heading of 0 degrees
Movement myMovement = MovementBuilder.moveThrough(12, 24, 0).build();
```
## More Advanced Modular Movements

```java
// Moves the robot 24 inches in the y direction at a heading of 90 degrees and stops. When the linearSlide is lowered it will turn off the power. Passing an argument into `.waitForMovement` is optional but if you do it will run the function every loop.
new Movement(0, 24, 90)
    .stopAtPosition()
    .waitForMovement(() -> {
        if (linearSlide.isLowered()) {
            linearSlide.setPower(0);
        }
    });
```
## Following Paths

### Bezier Curves

### Lines

pedro path - requires lots of tuning
pure pursuit - always goes at full power - no velocity constraints
roadrunner - doesn't react real-time until near target

current velocity divided by maxVelocity

TODO feautes +Kv = 1/maxVelocity +Kstatic minimum amount to move
Autotuning constants
Autotuning braking distance over time

## Why Did We Develop Black Ice?

- To intuitively learn, hands-on, how path following works in robotics. We encourage users to experiment by creating their own correction functions or adding new features.
- To explore more efficient and simpler solutions
- To minimize the tuning process and reduce complexity
- To have more modular customization

### We Encourage *You* to Make Your Own Movement System

We developed Black Ice to intuitively learn, hands-on, how path following works in robotics. We encourage users to do the same by experimenting with their own correction functions or adding new features. We designed Black Ice very modular in order to easily change the path-following algorithms.

Here are some ideas for new features:

- Try following paths using derivatives (functions that give you the slope of a curve at any point) of the bezier curves. This may have to be combined with a translational correction to stay on the path. This is what Pedro Path uses, however they do not incorporate braking distances.
- Implement a better target velocity algorithm.
- Implement feedforward system.

# How Does Black Ice Work?

## Predicting the Directional Braking Distance

Black Ice has 2 automatic tuning opModes (one forward/backward and one lateral) that makes the robot travel at different velocities. Once it reaches a certain velocity, it sets the power to 0 with zero power brake mode on. This locks the wheels and makes the robot stop. It then calculates the distance it took the robot to stop. Once it gets a certain amount of points it uses a quadratic regression algorithm to derive a quadratic formula that predicts the robot's braking distance at any velocity.
We use the constants it gives and plug it into a signed-quadratic function (which means that works in both positive and negative numbers). [x is the xVelocity or yVelocity]
We have a linear x term because through our tests we found velocity to braking distance to not be solely quadratic.

$$
f(x) = \ a \cdot x \cdot \text{abs}(x) + b \cdot x + c \cdot \text{sgn}(x) \
$$

```java
public double predict(double x) {
    return (a * x * Math.abs(x)) + (b * x) + (c * Math.signum(x));
}
```
## Stopping at a Position

`(error - predictedBrakingDistance) * constant`
`targetPosition - (currentPosition + predictedBrakingDisplacement)`
When velocity and acceleration constraints are needed it switches to a PIDF controller.
`kP * velocityError + kD * acceleration + feedforward`
`feedforward = kStatic + kP * targetVelocity` (incorportate voltage?)
Create a motion profile based of acceleration and deacceration
`p = kP * (error - braking) * 1 + kDervivative?`

The braking distance is calculated in real time and the numbers can be negative. The robot will travel at maximum speed until the braking term overpowers the error. This will make the robot stop

For moveThrough position set the magnitude to 1 (same thing as we are doing up allow upscaling)

## Following Paths

Black Ice traces out points on a Bezier Curve for the robot to follow. This forces the robot to follow the exact path.
The robot drives at full power towards these points. It knows whether it is past a point if the robot's predicted position is past the plane perpendicular to the line between the two points.

By having the robot follow points, this forces the robot to follow the path. Using dervivates like pedro paths adds complexity and adds transitional correction.

Support for Bezier Curves

HIgh proportianl
Black Ice is a efficient and effective dynamic path follower developed by Team #18535, Frozen Code, in the 2025 season and offseason.

How is ours is different from others like Roadrunner and Pedro Path? The key difference is that Black Ice calculates braking distance by having the motors on zero power brake mode. Other libraries, calculate drift distance by having the motors on zero power float mode. Our framework uses the braking distance to predict where the robot will be next, allowing it to maintain full speed until it reaches an optimal braking point, minimizing unnecessary slowing while ensuring precise stopping.

We have just one tuning test that runs the robot at different velocities to brake and calculate the braking distance. With those data points, we used quadratic regression (since stopping distance is proportional to velocity squared) to derive a function that accurately predicts the required braking distance at any speed.

Our drive power formula is purely proportional, subtracting the predicted braking distance from the target position. No arbitrary constants are needed. No integral term is needed, as stationary robots have zero braking distance. No derivative term is required either, since braking distance naturally adjusts to slow the robot precisely when needed.

# Credits

All code developed by Jacob Ophoven with help of Coach Andy and members of the FTC community.
Early Discord Supporters: alexreman45

Smoother Transitions: Ensuring less jerky motions for precision-intensive tasks.
Faster Execution: Reducing time spent in path planning and execution.
Dynamic Adjustments: Reacting to obstacles or changes in the environment in real-time.
