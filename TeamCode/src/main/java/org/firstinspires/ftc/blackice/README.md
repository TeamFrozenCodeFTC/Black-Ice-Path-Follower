# {🧊} Black Ice Path Follower - 5x Faster Deceleration

Black Ice (by FTC Team #18535, Frozen Code) is a **reactive + predictive path follower** that adapts in real-time along paths.

It drives at **high speeds** and then brakes up to **5× faster** than other libraries.

Black Ice achieves this by using [**quadratic-damped PIDs**](https://github.com/TeamFrozenCodeFTC/Black-Ice-Path-Follower/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/blackice/docs/quadratic-damping-pid.md#our-key-innovation-the-quadratic-damped-pid), which model nonlinear friction and resistance. These effects cause real braking distances to scale closer to velocity² rather than velocity under active braking. This lets it accurately predict stopping behavior at any speed, preventing the overshoot and undershoot common with aggressive PIDs that don't account for friction under braking. This allows much faster, more accurate, and more responsive control by braking to a stop rather than coasting to a stop like other libraries such as Roadrunner or Pedro Path.

## Features
- **High-speed** routines with optional smoother deceleration
- **Command system** using `.runAction()` in path routines
- **Modular customization** of paths
- **Less boilerplate** for creating path routines

## Requirements
- **Odometry wheels** (or other localization for pose + heading)
- **Mecanum wheels** (other drivetrains supported if you implement the Drivetrain interface)
- **Java + Android Studio**

## How much difference does the Quadratic Damping make?
Other libraries (e.g. Roadrunner, Pedro Path) rely on gentle PID + coasting:

- ~`-40 in/s²` → needs ~`45in` to stop from `60 in/s`

With quadratic damping + back-EMF braking:

- ~`10 in` to stop from `60 in/s` → nearly **5× faster deceleration**
- Leaves more time for acceleration before braking

### Pros
- Faster - more time accelerating, less time decelerating
- More accurate - higher proportional constant reduces steady-state error
- More responsive - reacts quickly without oscillation due to quadratic damping

### Trade-offs
- Higher power usage - requires more voltage, applying reverse power to brake faster
- Sharper stops - can feel less smooth (minor shaking at final stop)


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

# Credits

All code developed by Jacob Ophoven with help of Coach Andy and members of the FTC community.
Thanks to Pedro Path developers, especially Havish from 12808 RevAmped Robotics, for supporting my project and wanting to implement it into Pedro Path.
I am the lead on deceleration for upcoming v1.2.0 Pedro Path that we plan to implement my quadratic-damped PIDs for the translational vector.
I am also a beta tester that have helped out Pedro Path.
