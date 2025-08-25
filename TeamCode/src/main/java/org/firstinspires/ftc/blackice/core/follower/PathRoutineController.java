package org.firstinspires.ftc.blackice.core.follower;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.util.actions.ActionLoop;
import org.firstinspires.ftc.blackice.core.hardware.localization.MotionTracker;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;
import org.firstinspires.ftc.blackice.core.paths.Path;
import org.firstinspires.ftc.blackice.core.paths.routines.RoutineStep;
import org.firstinspires.ftc.blackice.core.paths.geometry.PathPoint;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.Drivetrain;
import org.firstinspires.ftc.blackice.util.Advancer;
import org.firstinspires.ftc.blackice.util.Logger;

public class PathRoutineController {
    private final DrivePowerController drivePowerController;
    private final MotionTracker motionTracker;
    private final FollowingTimeouts followingTimeouts = new FollowingTimeouts();
    
    private FollowingState followingState = FollowingState.IDLE;
    private Advancer<RoutineStep> routineAdvancer;
    private MotionState motionState;
    private PathState pathState;
    private PathPoint closestPathPoint;
    private boolean isBraking = false;
    private double previousPathPointTValue;
    private @Nullable Path currentPath;
    
    public void useCentripetal(boolean useCentripetal) {
        drivePowerController.useCentripetal = useCentripetal;
    }
    
    public void useDrive(boolean useDrive) {
        drivePowerController.useDrive = useDrive;
    }
    
    public void useTranslational(boolean useTranslational) {
        drivePowerController.useTranslational = useTranslational;
    }
    
    public void useHeading(boolean useHeading) {
        drivePowerController.useHeading = useHeading;
    }
    
    public void setHeadingPID(boolean useHeading) {
        drivePowerController.useHeading = useHeading;
    }
    
    public enum FollowingState {
        /** The follower is currently following a path. */
        FOLLOWING,
        /** The follower is at the end of the path holding. is waiting for something before it can be done with the path. */
        HOLDING,
        /** The follower is done with the path. Holds the path's end point until you stop looping. */
        DONE,
        /** The follower is temporarily paused. This stops the robot and stops applying power to the drivetrain. */
        PAUSED,
        /** The follower's path was either canceled or the follower is currently not
         * following any paths. */
        IDLE
    }
    
    PathRoutineController(
        DrivePowerController drivePowerController,
        MotionTracker motionTracker
    ) {
        this.drivePowerController = drivePowerController;
        this.motionTracker = motionTracker;

        updateMotionState();
    }
    
    public Drivetrain getDrivetrain() {
        return drivePowerController.drivetrain;
    }
    
    private void updateMotionState() {
        motionTracker.update();
        motionState = motionTracker.getMotionState();
    }
    
    /**
     * Set the robot's localizer to the given pose.
     * Also updates the path constructor's current pose so next path starts from here
     * This is useful to reset the robot's position during a match if you know exactly where it is.
     */
    public void setCurrentPose(Pose pose) {
        motionTracker.setCurrentPose(pose);
    }
    
    public void setCurrentHeading(double heading) {
        motionTracker.setCurrentHeading(heading);
    }
    
    public Path getCurrentPath() {
        if (currentPath == null) {
            throw new IllegalStateException("No path being followed.");
        }
        return currentPath;
    }
    
    /**
     * Add on a path routine to be followed after finishing the current path routine.
     */
    public void queueNext(PathRoutine pathRoutine) {
        routineAdvancer.extendQueue(pathRoutine.getSteps());
    }
    
    /**
     * Starts following a PathRoutine. Overrides any previous unfinished pathRoutine.
     * Call follower.update() to continue following.
     */
    public void startFollowing(PathRoutine pathRoutine) {
        routineAdvancer = new Advancer<>(pathRoutine.getSteps());
        startNextPath();
    }
    
    private ActionLoop getCurrentActionLoop() {
        return getCurrentPath().behavior.actionLoop;
    }
    
    /**
     * Cancels the current path and moves on to the next path or action in the routine.
     */
    public void cancelCurrentPath() {
        if (currentPath == null) {
            return;
        }
        
        getCurrentActionLoop().cancel();
        startNextPath();
        
        if (followingState == FollowingState.DONE) {
            stop();
        }
    }
    
    /**
     * Finishes the current path and moves on to the next path or action in the routine.
     */
    public void earlyExitCurrentPath() {
        if (currentPath == null) {
            return;
        }
        getCurrentActionLoop().finish();
        startNextPath();
    }
    
    private boolean tryAdvanceToNextStep() {
        if (!routineAdvancer.advance()) {
            followingState = FollowingState.DONE;
            followingTimeouts.done();
            return false;
        }
        return true;
    }
    
    private void startNextPath() {
        if (!tryAdvanceToNextStep()) return;
        
        RoutineStep step = routineAdvancer.current().resolve();
        while (step instanceof Action) {
            ((Action) step).execute();
            if (!tryAdvanceToNextStep()) return;
            step = routineAdvancer.current().resolve();
        }
        
        start((Path) step);
    }
    
    private void start(Path path) {
        currentPath = path;
        
        followingState = FollowingState.FOLLOWING;
        isBraking = false;
        previousPathPointTValue = 0;
        followingTimeouts.start();
        
        getCurrentActionLoop().start();
    }
    
    /**
     * Is idle when the follower is not following any path routine.
     * <p>
     * This can happen when the follower has not started any paths or is canceled using
     * {@link #stop()} or if {@link #cancelCurrentPath()} has been used on the last path.
     */
    public boolean isIdle() {
        return followingState == FollowingState.IDLE;
    }
    
    /**
     * Whether the follower is currently advancing along the path routine.
     * <p>
     * Note: Is false when paused or holding.
     */
    public boolean isFollowing() {
        return followingState == FollowingState.FOLLOWING;
    }
    
    /**
     * Returns true if the robot is currently outputting power to the drivetrain.
     * This includes FOLLOWING, HOLDING, and DONE; all cases where the robot is trying
     * to move or maintain a pose.
     * <p>
     * This used in TeleOp so you can check when you can apply manual power.
     * <p>
     * NOTE: This may be true *after* the path routine is finished (e.g., DONE holding position).
     * Use {@link #isInProgress()} to check whether path execution is still in progress.
     */
    public boolean isCommandingPower() {
        return isFollowing() || isDone() || isHolding();
    }
    
    /**
     * Whether the follower is currently holding at the end of the current path.
     */
    public boolean isHolding() {
        return followingState == FollowingState.HOLDING;
    }
    
    public boolean isPaused() {
        return followingState == FollowingState.PAUSED;
    }
    
    public boolean isDone() {
        return followingState == FollowingState.DONE;
    }
    
    /**
     * Stops following all path routines. Turns the robot on zero power brake mode and brakes.
     */
    public void stop() {
        followingState = FollowingState.IDLE;
        drivePowerController.brakeWithZeroPowerBrakeMode();
        followingTimeouts.done();
        
        if (currentPath != null) {
            getCurrentActionLoop().cancel();
        }
    }
    
    /**
     * Pauses the current path routine if following. Turns the robot on zero power brake
     * mode
     * and brakes.
     * @see #resume
     * @return true/false whether or not the pause was successful
     */
    public boolean pause() {
        if (!isCommandingPower() || currentPath == null) {
            return false;
        }
        followingState = FollowingState.PAUSED;
        drivePowerController.brakeWithZeroPowerBrakeMode();
        followingTimeouts.pause();
        getCurrentActionLoop().pause();
        return true;
    }
    
    /**
     * Resumes the current path routine if paused.
     * @see #pause
     * @return true/false whether or not the resume was successful
     */
    public boolean resume() {
        if (followingState != FollowingState.PAUSED) {
            return false;
        }
        followingState = FollowingState.FOLLOWING;
        followingTimeouts.resume();
        getCurrentActionLoop().resume();
        return true;
    }
    
    /**
     * Returns true if the follower is actively running a path routine.
     * This means the follower is not yet finished and has not been canceled.
     */
    public boolean isInProgress() {
        return followingState != FollowingState.DONE && followingState != FollowingState.IDLE;
    }
    
    /**
     * Updates the robot's motionState and gives power to motors to follow the path if a path
     * routine is set. Call this once every loop.
     * <pre><code>
     * while (follower.isBusy()) {
     *     follower.update();
     * } // this will continue to follow the path routine until it is finished or canceled.
     * </code></pre>
     */
    public void update() {
        updateMotionState();
        
        if (isIdle()) return;
        
        getCurrentActionLoop().loop();
        
        if (followingTimeouts.hasTimedOut(getCurrentPath().behavior, motionState, isFollowing())) {
            cancelCurrentPath();
        }
        
        if (isPaused() || isIdle()) return;
        
        while (true) {
            follow(motionState);
            boolean currentPathIsComplete =
                getCurrentActionLoop().hasCanceled() || getCurrentActionLoop().hasFinished();
            if (!currentPathIsComplete) {
                break;
            }
            if (routineAdvancer.isDone()) {
                followingState = FollowingState.DONE;
                followingTimeouts.done();
                break;
            }
            startNextPath();
        }
        Logger.debug("XXX currentPosition", motionState.position);
        Logger.debug("XXX closestPointToRobot", closestPathPoint.point);
    }
    
    private void follow(MotionState motionState) {
        closestPathPoint = getCurrentPath().geometry.computeClosestPathPointTo(
            motionState.nextPosition,
            previousPathPointTValue
        );
        previousPathPointTValue = closestPathPoint.tValue;
        
        pathState = new PathState(getCurrentPath(), closestPathPoint, motionState);
        
        Logger.debug("distanceRemaining", closestPathPoint.distanceRemaining);
        Logger.debug("nextPosition", motionState.nextPosition);
        
        DrivePowerController.BrakingStatus brakingStatus =
            drivePowerController.isBraking(pathState);
        
        if (brakingStatus.isBraking || isBraking) {
            Logger.debug("isBraking -------------------");
            if (getCurrentPath().behavior.stop == PathBehavior.StopMode.NONE) {
                drivePowerController.drivetrain.zeroPower();
                getCurrentActionLoop().finish();
                return;
            }
            isBraking = true;
            if (hasStoppedAtSegmentEnd(pathState.motionState)) {
                if (getCurrentActionLoop().canFinish()) {
                    getCurrentActionLoop().finish();
                    return;
                }
                followingState = FollowingState.HOLDING;
            }
        }
        
        drivePowerController.drive(pathState, brakingStatus);
    }
    
    public FollowingState getFollowingState() {
        return followingState;
    }
    
    public MotionState getMotionState() {
        return motionState;
    }
    
    public PathPoint getClosestPathPoint() {
        return closestPathPoint;
    }
    
    /**
     * Returns true if the follower is braking to a stop or is holding a pose.
     */
    public boolean isBraking() {
        return isBraking;
    }
    
    private boolean hasStoppedAtSegmentEnd(MotionState motionState) {
        boolean isStopped =
            motionState.speed < getCurrentPath().behavior.stoppedVelocityConstraint
                && motionState.angularVelocity <
                getCurrentPath().behavior.stoppedAngularVelocityConstraint;
        boolean isAtEnd =
            pathState.closestPathPoint.percentAlongPath >= 0.995; // make this a constraint
        return isAtEnd && isStopped;
    }
}
