package org.firstinspires.ftc.blackice.core.follower;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutineBuilder;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.Drivetrain;
import org.firstinspires.ftc.blackice.FollowerConstants;
import org.firstinspires.ftc.blackice.util.Logger;

import java.util.List;

/**
 * Orchestrator
 */
public class Follower extends PathRoutineController {
    private static Follower INSTANCE;
    private static Pose lastOpModePose;
    
    public static Follower getInstance() {
        return INSTANCE;
    }
    public static Pose getLastOpModePose() {
        return lastOpModePose;
    }
    
    private PathBehavior defaultPathBehavior;
    
    public final Drivetrain drivetrain;
    
    private final Pose startingPose;
    
    private final double pauseWhenVoltageBelow;
    private final List<VoltageSensor> voltageSensors;
    
    private boolean lowVoltage = false;
    private double voltage = 0;
    
    public void addDefaultPathBehavior(PathBehavior behavior) {
        defaultPathBehavior = defaultPathBehavior.mergeWith(behavior);
    }
    
    public Follower(
        HardwareMap hardwareMap,
        FollowerConfig config,
        Pose startingPose
    ) {
        super(new DrivePowerController(
                config.headingPID,
                config.positionalPID,
                config.driveVelocityPIDF,
                config.centripetalFeedforward,
                config.maxReversalBrakingPower,
                config.drivetrainConfig.build(hardwareMap)
            ), config.localizerConfig.createMotionTracker(hardwareMap)
        );
        this.drivetrain = getDrivetrain();
        INSTANCE = this;
        this.startingPose = startingPose;
        setCurrentPose(startingPose);

        this.defaultPathBehavior = config.defaultPathBehavior;
        this.pauseWhenVoltageBelow = config.stopIfVoltageBelow;
        this.voltageSensors = hardwareMap.getAll(VoltageSensor.class);
    }
    
    public Follower(HardwareMap hardwareMap, Pose startingPose) {
        this(hardwareMap, FollowerConstants.defaultFollowerConfig, startingPose);
    }
    
    /**
     * Initializes the follower at Pose(0,0,0)
     */
    public Follower(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose(0,0,0));
    }
    
    public Pose getCurrentPose() {
        return getMotionState().pose;
    }
    
    /**
     * Returns a new {@link PathRoutineBuilder} with the default path behavior of this follower.
     * <p>
     * This is the way to create paths.
     */
    public PathRoutineBuilder pathRoutineBuilder() {
        return new PathRoutineBuilder(this::getCurrentPose)
            .withStartPose(startingPose)
            .withDefaultBehavior(defaultPathBehavior);
    }
    
    public PathRoutineBuilder pathRoutineBuilder(Pose startPose) {
        return pathRoutineBuilder()
            .withStartPose(startPose);
    }
    
    private void logVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        if (result < pauseWhenVoltageBelow && !lowVoltage) {
            pause();
            lowVoltage = true;
            Logger.warn("LOW VOLTAGE:  " + result + ". PAUSING FOLLOWER");
        }
        else if (result < 10 && lowVoltage) {
            resume();
            lowVoltage = false;
            Logger.warn("VOLTAGE BACK UP TO " + result + ". RESUMING FOLLOWER");
        }
        else {
            Logger.verbose("Voltage: ", result);
        }
        voltage = result;
    }
    
    public void update() {
        logVoltage();
        super.update();
    }
    
    public double getVoltage() {
        return voltage;
    }

    /**
     * Initializes the robot for tele-op mode,
     * using the position from the end of the autonomous period.
     */
    public void initTeleOp() {
        setCurrentPose(lastOpModePose);
        drivetrain.zeroPowerBrakeMode();
    }

    public void savePoseForTeleOp() {
        lastOpModePose = new Pose(
            getMotionState().position,
            Math.toDegrees(getMotionState().heading)
        );
    }

    /**
     * Run a basic field-centric tele-op.
     * <pre><code>
     * follower.fieldCentricTeleOpDrive(
     *     -gamepad1.left_stick_y
     *     -gamepad1.left_stick_x,
     *     -gamepad1.right_stick_x
     * );
     * </code></pre>
     */
    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
        drivetrain.followVector(getMotionState().makeRobotRelative(new Vector(forward,
                                                                              lateral))
            , turn, true);
    }

    public void robotCentricDrive(double forward, double lateral, double turn) {
        drivetrain.followVector(new Vector(forward, lateral), turn, true);
    }
}
//    public void logVoltage() {
//        double voltage = getVoltage();
//        Logger.verbose("voltage", voltage);
//        if (voltage < 7) {
//            Logger.warn("LOW VOLTAGE WARNING of " + voltage + "v. CRASH INBOUND");
//            Logger.warn("PREVENTING CRASH. CANCELING FOLLOWER");
//            getCurrentActionLoop().cancel();
//        }
//    }
    


//    private final ActionLoop actionFollowingLoop = new ActionLoop();
//
//    /**
//     * Continues and hold's the current path end pose until the condition is true.
//     * <pre><code>
//     * .holdUntil(() -> robot.getLiftPosition() > 100) // hold until lift is above 100
//     * </code></pre>
//     * If the condition is true before the path is finished, the robot will continue until it
//     * reaches the end of the path.
//     */
//    public Follower holdUntil(Condition conditionIsTrue) {
//        getCurrentPathExecutor().getActionLoop().canFinishWhen(conditionIsTrue);
//        waitForPath();
//        return this;
//    }
//
//    /**
//     * Cancel and stop following all paths when the condition is true.
//     * <pre><code>
//     * follower.cancelAllWhen(() -> robot.getLiftPosition() > 100) // stop when lift is above 100
//     * </code></pre>
//     * Will not return to following paths until the condition is false.
//     */
//    public Follower cancelAllWhen(Condition condition) {
//        drivetrain.zeroPowerBrakeMode();
//        drivetrain.zeroPower();
//        actionFollowingLoop.cancelWhen(condition);
//        return this;
//    }
//    /**
//     * Cancel and stop following all paths when the condition is true and execute the action.
//     * <pre><code>
//     * follower.cancelAllWhen(gamepad1::x, gamepad1::rumble); // rumble the controller when X is pressed and cancel
//     * </code></pre>
//     */
//    public Follower cancelAllWhen(Condition condition, Action action) {
//        drivetrain.zeroPowerBrakeMode();
//        drivetrain.zeroPower(); // this needs to be in the actual action
//        actionFollowingLoop.cancelWhen(condition, action);
//        return this;
//    }
//
//    // update
////    localizer.update();
////    motionTracker.update();
////    pathExecutor.updateDrivePowersToFollowPath(...);
////
////    if (actionLoop != null && actionLoop.isRunning()) {
////        actionLoop.loop();
////    }
//
//    /**
//     * Calls the given action every loop while a path is being followed.
//     * <pre><code>
//     * follower.onUpdate(() -> myLift.updatePosition()) // update lift position while following
//     * </code></pre>
//     */
//    public Follower whileFollowing(Action action) {
//        actionFollowingLoop.onLoop(action);
//        return this;
//    }
//
//    /**
//     * Executes the action when the condition is true.
//     * <pre><code>
//     * follower.doWhen(gamepad1::a, slide::raise); // raise slide whenever A is pressed
//     * </code></pre>
//     */
//    public Follower doWhen(Condition condition, Action executable) {
//        return this.whileFollowing(() -> executable.executeWhen(condition.isTrue()));
//    }
//
//    // Not very many use cases for these:
//    /**
//     * Add an action to be executed when a path starts.
//     */
//    public Follower onPathStart(Action action) {
//        actionFollowingLoop.onStart(action);
//        return this;
//    }
//    /**
//     * Add an action to be executed when the path successfully finishes. This includes early exits
//     * but not cancellations.
//     */
//    public Follower onPathFinish(Action action) {
//        actionFollowingLoop.onFinish(action);
//        return this;
//    }
//    /**
//     * When a path is canceled, either by an opMode stop, or other conditions like macro
//     * cancelling buttons.
//     */
//    public Follower onPathCancel(Action action) {
//        actionFollowingLoop.onCancel(action);
//        return this;
//    }
//    /**
//     * When the path is exited, either by successfully finishing or canceling.
//     */
//    public Follower onPathExit(Action action) {
//        actionFollowingLoop.onExit(action);
//        return this;
//    }
//
//    public void waitUntilOpModeStop() {
//        waitUntil(Condition.NEVER);
//    }
//
////    public void waitUntil(Condition condition) {
////        while (opMode.opModeIsActive() && !condition.isTrue()) {
////            opMode.idle();
////        }
////    }
//
//    /**
//     * Initializes the robot for tele-op mode,
//     * using the position from the end of the autonomous period.
//     */
//    public void initTeleOp() {
//        localizer.setPose(lastOpModePose);
//        motionTracker.update();
//        motionState = motionTracker.getMotionState();
//        drivetrain.zeroPowerBrakeMode();
//    }
//
//    public void savePoseForTeleOp() {
//        lastOpModePose = new Pose(
//            motionState.position,
//            Math.toDegrees(motionState.heading)
//        );
//    }
//
//    public void setHeadingResetButton(Condition gamepadCondition, double headingDegrees) {
//        this.doWhen(
//            gamepadCondition,
//            () -> localizer.setHeading(headingDegrees)
//        );
//    }
//
//    public void setPoseResetButton(
//        Condition gamepadCondition,
//        double resetX, double resetY, double resetHeading
//    ) {
//        this.doWhen(
//            gamepadCondition,
//            () -> localizer.setPose(resetX, resetY, resetHeading)
//        );
//    }
//
//    /**
//     * Run a basic field-centric tele-op.
//     * <p>
//     * For driver field-centric
//     * <pre><code>
//     * follower.fieldCentricTeleOpDrive(
//     *     -gamepad1.left_stick_y
//     *     -gamepad1.left_stick_x,
//     *     -gamepad1.right_stick_x
//     * );
//     * </code></pre>
//     */
//    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
//        updateMotionState();
//        if (opMode.gamepad1.right_stick_button){ // turning
//            drivetrain.applyBrakingPowers(motionState.makeRobotRelative(new Vector(forward,
//                    lateral)),
//                turn);
//        }
//        else { // simple add
//            drivetrain.followVector(motionState.makeRobotRelative(new Vector(forward, lateral)),
//                turn); // combine into one with condition
//        }
//    }
//
//    public void robotCentricDrive(double y, double x, double turn) {
//        updateMotionState();
//        drivetrain.followVector(new Vector(x, y), turn);
//    }

//
//    private FollowingState handleBraking() {
//        if (!pathAdvancer.isDone()) {
//            advance();
//            return FollowingState.FOLLOWING;
//        }
//        if (getCurrentPath().behavior.stopAtEnd) {
//            return FollowingState.BRAKING;
//        }
//        // does not listen to actionLoop canFinish because the
//        // robot is keeping it's momentum and cannot hold a point
//        combinedLoop.finish();
//        return FollowingState.DONE; // go to next path as fast as possible
//    }
//
//    private FollowingState applyBraking(DrivePowerController.BrakingStatus brakingStatus) {
//
//        if (hasStoppedAtSegmentEnd(pathState.motionState)) { // + is within error margin +
//            // heading is aligned OR when drive powers are less than 0.1 or something
//            state = FollowingState.HOLDING;
//
//            if (!pathAdvancer.isDone()) {
//                advance();
//                return FollowingState.FOLLOWING;
//            }
//            if (combinedLoop.canFinish()) {
//                combinedLoop.finish();
//                return FollowingState.DONE;
//            }
//            Logger.debug("holding -------------------");
//
//            // pause
//            // cancel = pause and restart state
//        }
//
//        drivePowersCalculator.drive(drivetrain, pathState, brakingStatus);
//
//        return state;
//    }
