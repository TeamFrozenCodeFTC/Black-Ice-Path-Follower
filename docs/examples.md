---
layout: on-this-page
title: "Examples"
nav_order: 2
---

# Examples

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