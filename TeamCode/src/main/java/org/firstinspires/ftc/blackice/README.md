# Black Ice Path Follower

Black Ice (by FTC Team #18535, Frozen Code) is a **Reactive and Predictive Path Follower** that adapts in real-time along paths. It drives at high speeds and then brakes **5x times faster** than other path following libraries which coast to a stop.
It does this by triggering back-EMF with **quadratic-damped PIDs** to model the non-linear effects of friction and momentum. This lets it accurately predict braking at any velocity, preventing the overshoot and undershoot common with regular aggressive PIDs. This allows much faster, more accurate, and more aggressive control by braking to a stop rather than coasting to a stop like other libraries such as Roadrunner or Pedro Path.

Tailored for teams who want:
- High-speed and aggressive path routines
- Minimal boilerplate for creating path routines
- Modular customization of paths
- Command system using `.runAction()` in routines

Requirements:

- **Odometry Wheels** or some form of localization that can determine the robot's position and heading.
- Currently requires **mecanum wheels**, but there is support for other drivetrains if you implement the Drivetrain interface.

# Example Usages

## LinearOpMode Example

```java
@Autonomous
public class Example extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap, new Pose(0, 0, 0));

        PathRoutine path = follower.pathRoutineBuilder()
            .lineTo(48, 0)
            .runAction(() -> {
                telemetry.addLine("Reached (48, 0)");
                telemetry.update();
            })
            .lineTo(48, 24)
                .withHeadingInterpolationTo(90)
            .stop()
            .build();
        
        waitForStart();
        
        follower.startFollowing(path);

        while (opModeIsActive()) {
            follower.update();
        }
    }
}

```



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
Thanks to Pedro Path developers, especially Havish from 12808 RevAmped Robotics, for supporting my project and wanting to implementing.
We are project lead on decel on pedro path as well as beta testers.

Smoother Transitions: Ensuring less jerky motions for precision-intensive tasks.
Faster Execution: Reducing time spent in path planning and execution.
Dynamic Adjustments: Reacting to obstacles or changes in the environment in real-time.
