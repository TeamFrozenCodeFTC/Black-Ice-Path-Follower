---
layout: toc2
title: "Overview"
description: "What is Black Ice?"
nav_order: 1
has_toc: true
has_children: true
---


# {🧊} Black Ice Path Follower - 5x Faster Deceleration

## Table of Contents
{: .no_toc .text-delta }


Black Ice (by FTC Team #18535, Frozen Code) is a **reactive + predictive path follower** that adapts in real-time along paths.

It drives at **high speeds** and then brakes up to **5× faster** than other libraries.

Black Ice achieves this by using [**quadratic-damped PIDs**](https://github.com/TeamFrozenCodeFTC/Black-Ice-Path-Follower/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/blackice/docs/quadratic-damping-pid.md#our-key-innovation-the-quadratic-damped-pid), which model nonlinear friction and resistance. This lets it accurately predict stopping behavior at any speed, preventing the overshoot and undershoot common with aggressive PIDs that don't account for friction while braking. This allows much **faster**, more **accurate**, and more **responsive** control by braking to a stop rather than coasting to a stop like other libraries such as Roadrunner or Pedro Path.

## How much difference does the Quadratic Damping make?
Other libraries (e.g. Roadrunner, Pedro Path) rely on gentle PID + coasting:

- Coasting (~`-40 in/s²`) needs **~`45in`** to stop from `60 in/s`

With quadratic damping + back-EMF braking:

- **~`10 in`** to stop from `60 in/s` → nearly **5× faster deceleration**
- Leaves more time to accelerate before braking

### Pros
{: .no_toc }
- **Faster** - more time accelerating, less time decelerating
- **More accurate** - higher proportional constant reduces steady-state error
- **More responsive** - reacts quickly without oscillation due to quadratic damping

### Trade-offs
{: .no_toc }
- **Higher power usage** - requires more voltage, applying reverse power to brake faster
- **Sharper stops** - can feel less smooth (minor shaking at final stop)

## Features
- **High-speed** routines (made of BezierCurves, Lines, and Points) with optional smoother deceleration
- **Command system** using `.runAction()` in path routines
- **Modular customization** of paths
- **Less boilerplate** for creating path routines

## Requirements
- **Odometry wheels** (or other localization for pose + heading)
- **Mecanum wheels** (other drivetrains supported if you implement the Drivetrain interface)
- **Java + Android Studio**


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

## Pauseable TeleOp Macro
```java
/**
 * This opMode runs a field centric tele-op with a macro that moves the robot from it's current pose to (0,0).
 * If the driver tries to move the joysticks while the macro is running,
 * it will pause the macro and allow manual input until the macro is started again or resumed.
 */
@TeleOp
public class TeleOpMacro extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        // A path that goes from the current pose to (0,0) and stops.
        PathRoutine macro = follower.pathRoutineBuilder()
            .fromCurrentPose()
            .toPose(0, 0)
            .stop()
            .build();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Starts the macro that goes to (0,0) when dpad down is pressed
            if (gamepad1.dpadDownWasPressed()) {
                follower.startFollowing(macro);
            }
            
            follower.update();
            boolean isTryingToMove =
                Math.abs(gamepad1.left_stick_y) > 0.5 || Math.abs(gamepad1.left_stick_x) > 0.5;
            
            // Pauses the follower's path if the driver is trying to move the robot's manually with the joysticks
            if (isTryingToMove) {
                follower.pause();
            }
            // Resumes the follower's path
            else if (gamepad1.x) {
                follower.resume();
            }
            
            // It is safe to supply manual teleOp power when the follower is not commanding the robot
            if (!follower.isCommandingPower()) {
                follower.fieldCentricTeleOpDrive(
                    -gamepad1.left_stick_y, // forward
                    -gamepad1.left_stick_x, // strafe
                    -gamepad1.right_stick_x // turn
                );
            }

            telemetry.addData("state", follower.getFollowingState());
            telemetry.update();
        }
    }
}
```

# Why Did We Develop Our Own Path Follower?

1. We wanted to create a **faster** path follower that could brake as fast as `ZeroPowerBrakeMode` when needed and did not have to always coast to a stop.
   - This was because our goal for the 2024–2025 season was a five-specimen autonomous using only 312 RPM wheels and after scoring each specimen, the robot had to turn around. 
   - To achieve this, we needed our autonomous to be as fast as possible; something we couldn’t accomplish without having our deceleration as fast as `ZeroPowerBrakeMode`.
2. We were **interested** in path following, and we wanted to intuitively **learn**, hands-on, how path following works.
3. We wanted more **customization**, modularization, and less boilerplate when building paths and autonomous routines.

It was worth creating our own path follower because it led us to add a quadratic-damping term to our translational PID. This made the system nearly five times faster at decelerating. We never would have discovered this approach without building our own follower with all the eariler prototype and iterations. Setting the goal of stopping as quickly and accurately as possible, using Zero Power Brake Mode as inspiration, pushed us to that breakthrough.

# Credits

All code developed by **Jacob Ophoven** with help of **Coach Andy** and members of the FTC community.
Thanks to Pedro Path developers, especially **Havish from 12808 RevAmped Robotics**, for supporting my project and wanting to implement it into Pedro Path (an advanced path follower used by teams at worlds).

We have also given back to Pedro Path, our contributions to Pedro Path so far for v1.1.0 include:
- HeadingInterpolator interface
- BezierCurve.through that creates a beizer curve that goes through the given points
- Kinematics class
- Wrote the Docs for pedro's deceleration algorithm
- Tested their new Bezier Curve class that uses Matrices

We are on the lead of deceleration for upcoming v1.2.0 Pedro Path and we plan to implement our quadratic-damped PIDs for the translational vector.
In the mean time, we will create a separate fork of Pedro Path with our the Black Ice follower but with all of the access to Pedro's localization and tuning.

