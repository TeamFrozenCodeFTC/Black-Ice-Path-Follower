package org.firstinspires.ftc.blackice.core.follower;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.blackice.core.control.PIDController;
import org.firstinspires.ftc.blackice.core.control.VelocityController;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.Drivetrain;
import org.firstinspires.ftc.blackice.util.Logger;
import org.firstinspires.ftc.blackice.util.Validator;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Uses error controllers to convert target velocities and positions into vectors and
 * motor powers to be driven.
 */
class DrivePowerController {
    /**
     * Responsible for turning the robot and making sure it is facing the correct
     * direction.
     */
    final PIDController headingPID;
    
    /**
     * Responsible holding a given pose and giving translational power for the robot to
     * stay on the path. Tune this as aggressively as possible without the robot shaking
     * while holding a pose. Error is in distance from target point (inches).
     */
    final PIDController positionalPID;
    
    /**
     * Responsible accelerating and decelerating the robot's velocity.
     */
    final VelocityController driveVelocityController;
    
    boolean useCentripetal = true, useDrive = true, useTranslational = true,
        useHeading = true;
    public double centripetalScaling;
    
    public final Drivetrain drivetrain;
    private final double maxReversalBrakingPower;
    double lastDrivePower = 1;
    double totalError = 0;
    double iterations = 0;
    
    public DrivePowerController(PIDController headingPID,
                                PIDController positionalPID,
                                VelocityController drivePIDF, double centripetalScaling,
                                double maxReversalBrakingPower, Drivetrain drivetrain) {
        this.headingPID = headingPID;
        this.positionalPID = positionalPID;
        this.centripetalScaling = centripetalScaling;
        this.drivetrain = drivetrain;
        this.driveVelocityController = drivePIDF;
        this.maxReversalBrakingPower =
            Validator.ensurePositiveSign(maxReversalBrakingPower);
        
        Logger.initializeGraphKeys("currentVelocity", "targetVelocity", "velocityToLoseDueToMomentum",
                                   "brakingPower", "drivePower", "translationalPower",
                                   "centripetalPower", "turnPower", "totalPower",
                                   "path error", "finalCoastVelocity", "momentumCompensatedFeedforward");
    }
    
    /**
     * Prevents the robot from applying too much power in the opposite direction of the
     * robot's momentum. This prevents voltage drops and doesn't hurt braking performance
     * that much since reversing the direction of wheels with just -0.001 power behaves
     * identical to -0.3 at high speeds due to EMF braking.
     */
    private double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        
        if (!isOpposingMotion) {
            return power;
        }
        
        double clampedPower;
        if (power < 0) {
            clampedPower = Math.max(power, -maxReversalBrakingPower);
        } else {
            clampedPower = Math.min(power, maxReversalBrakingPower);
        }
        return clampedPower;
    }
    
    public Vector computeBrakingPower(PathState pathState) {
        Vector error =
            pathState.path.endPose.getPosition().minus(pathState.motionState.position);
        return error.map(pathState.motionState.velocity,
                         (err, vel) -> positionalPID.runFromError(err, -vel));
    }
    
    public Vector computeCentripetalVector(PathState pathState) {
        double centripetalForce =
            pathState.tangentialVelocity * pathState.tangentialVelocity *
                pathState.closestPathPoint.curvature * centripetalScaling;
        
        return pathState.closestPathPoint.perpendicularVector.withMagnitude(
            centripetalForce);
    }
    
    /**
     * Computes the amount of feedforward velocity to reach the end of the path, while
     * accounting for momentum.
     */
    private double computeMomentumCompensatedFeedforward(double targetVelocity,
                                                   PathState pathState) {
        // The velocity the robot would be at if it coasted to the end of the path.
        double finalCoastVelocity = Math.sqrt(Math.abs(
            pathState.tangentialVelocity * pathState.tangentialVelocity +
                (2 * -driveVelocityController.naturalDeceleration *
                    (pathState.closestPathPoint.distanceRemaining)))); // -12?
        
        // How much velocity the robot loses due to momentum while coasting to the end of the path.
        double velocityToLoseDueToMomentum =
            pathState.tangentialVelocity - finalCoastVelocity;
        
        // We want to stop the robot's momentum so we subtract the
        // velocityToLoseDueToMomentum from the target velocity.
        double addedFeedforwardVelocity =
            targetVelocity - velocityToLoseDueToMomentum;
        
        Logger.graph("momentumCompensatedFeedforward", addedFeedforwardVelocity);
        Logger.graph("finalCoastVelocity", finalCoastVelocity);
        Logger.graph("velocityToLoseDueToMomentum", velocityToLoseDueToMomentum);
        
        if (pathState.path.behavior.allowReversePowerDeceleration) {
            return addedFeedforwardVelocity;
        }
        return Math.max(0, addedFeedforwardVelocity);
    }

    public Vector computeDrivePowerVector(PathState pathState) {
        double drivePower;
        if (pathState.path.behavior.velocityProfile == null) {
            drivePower = 1;
        } else {
            double targetVelocity =
                pathState.path.behavior.velocityProfile.computeTargetVelocity(
                    pathState.closestPathPoint.distanceAlongPath,
                    pathState.closestPathPoint.distanceRemaining);
            
            double momentumCompensatedFeedforward =
                computeMomentumCompensatedFeedforward(targetVelocity, pathState);
            
            Logger.graph("currentVelocity", pathState.tangentialVelocity);
            Logger.graph("targetVelocity", targetVelocity);
            Logger.graph("momentumCompensatedFeedforward", momentumCompensatedFeedforward);
            
            drivePower = Range.clip(
                driveVelocityController.run(targetVelocity, pathState.tangentialVelocity,
                                            momentumCompensatedFeedforward), 0, 1);
        }
        lastDrivePower = drivePower;
        return pathState.closestPathPoint.tangent.withMagnitude(drivePower);
    }
    
    public Vector computeTranslationalPowerVector(PathState pathState) {
        Vector perpendicularDirection = pathState.closestPathPoint.perpendicularVector;
        
        Vector error =
            pathState.closestPathPoint.point.minus(pathState.motionState.nextPosition);
        
        double perpendicularError = error.dotProduct(perpendicularDirection);
        double perpendicularVelocity =
            pathState.motionState.nextVelocity.dotProduct(perpendicularDirection);
        double derivativeError = -perpendicularVelocity;
        double translationalPower =
            positionalPID.runFromError(perpendicularError, derivativeError);
        
        Logger.graph("path error", perpendicularError);
        totalError += Math.abs(perpendicularError);
        iterations++;
        Logger.debug("average path error", totalError / iterations);
        
        return perpendicularDirection.withMagnitude(translationalPower);
    }
    
    public BrakingStatus isBraking(PathState pathState, boolean hasBraked) {
        return new BrakingStatus(computeBrakingPower(pathState), pathState,
                                 lastDrivePower, hasBraked);
    }
    
    public void brakeWithZeroPowerBrakeMode() {
        drivetrain.brakeWithZeroPowerBrakeMode();
    }
    
    public void drive(PathState pathState, BrakingStatus brakingStatus) {
        if (brakingStatus.isBraking) {
            double turnPower = computeHeadingCorrectionPower(pathState);
            
            Logger.verbose("brakingPower", brakingStatus.brakingPower);
            
            drivetrain.followVector(pathState.motionState.makeRobotRelative(
                    brakingStatus.brakingPower.withMaxMagnitude(1))
                                        .map(pathState.motionState.robotRelativeVelocity,
                                             this::clampReversePower), turnPower, false);
            return;
        }
        
        // Prioritizes
        // 1. centripetal feedforward, to keep momentum on path curvature
        // 2. braking translational, to not overshoot between path transitions
        // 3. turning,
        // 4. tangential drive power
        
        // Translational is above turning because of its braking
        // responsibility.
        // All vectors have magnitudes <= 1 (mostly)
        
        double remainingSq = 1.0;
        
        Vector centripetalPower = new Vector(0, 0);
        Vector translationalPower = new Vector(0, 0);
        Vector drivePower = new Vector(0, 0);
        double turnPower = 0;
        
        if (useCentripetal) {
            centripetalPower = computeCentripetalVector(pathState);
            remainingSq = Math.max(0, remainingSq - centripetalPower.lengthSquared());
        }
        if (useTranslational && remainingSq > 0) {
            translationalPower = computeTranslationalPowerVector(pathState);
            remainingSq = Math.max(0, remainingSq - translationalPower.lengthSquared());
        }
        if (useHeading && remainingSq > 0) {
            turnPower = computeHeadingCorrectionPower(pathState);
            double maxTurn = Math.sqrt(remainingSq);
            turnPower = Math.copySign(Math.min(Math.abs(turnPower), maxTurn), turnPower);
            remainingSq = Math.max(0, remainingSq - turnPower * turnPower);
        }
        if (useDrive && remainingSq > 0) {
            drivePower = computeDrivePowerVector(pathState);
            Vector driveResidual =
                drivePower.minus(translationalPower.projectOnto(drivePower));
            double driveMag = driveResidual.computeMagnitude();
            double maxDriveMag = Math.min(driveMag, Math.sqrt(remainingSq));
            drivePower = driveResidual.withMagnitude(maxDriveMag);
        }
        
        Vector totalPower = centripetalPower.plus(translationalPower).plus(drivePower);
        
        Logger.verbose("centripetalPower", centripetalPower.computeMagnitude());
        Logger.verbose("translationalPower", translationalPower.computeMagnitude());
        Logger.verbose("turnPower", turnPower);
        Logger.verbose("drivePower", drivePower.computeMagnitude());
        Logger.verbose("totalPower", totalPower.computeMagnitude());
        Logger.updateGraph();
        
        Vector robotRelativePower = pathState.motionState.makeRobotRelative(totalPower)
            .map(pathState.motionState.robotRelativeVelocity, this::clampReversePower);
        
        drivetrain.followVector(robotRelativePower, turnPower, false);
    }
    
    public double computeHeadingCorrectionPower(PathState pathState) {
        double targetHeading =
            AngleUnit.RADIANS.normalize(pathState.path.headingInterpolator.interpolate(pathState.closestPathPoint));
        // may need to change turning direction if goes in wrong direction
        return Range.clip(headingPID.run(targetHeading, pathState.motionState.heading,
                                         pathState.motionState.deltaTime), -1, 1);
    }
    
    public static class BrakingStatus {
        public final boolean isBraking;
        final Vector brakingPower;
        
        BrakingStatus(Vector brakingPower, PathState pathState, double drivePower,
                      boolean hasBraked) {
            boolean isNearEnd = pathState.closestPathPoint.distanceRemaining <= 20;
            boolean isDecelerating = brakingPower.computeMagnitude() < drivePower;
            boolean isReversing =
                brakingPower.dotProduct(pathState.closestPathPoint.tangent) < 0;
            
            this.isBraking = hasBraked || (isNearEnd && (isDecelerating || isReversing));
            this.brakingPower = brakingPower;
        }
    }
}
