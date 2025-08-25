package org.firstinspires.ftc.blackice.core.control;


import org.firstinspires.ftc.blackice.util.Validator;

/**
 * A PIDF (Proportional-Integral-Derivative-Feedforward) controller that controls
 * velocity.
 * <p>
 * Does not use Integral or Derivative terms as they are usually unnecessary
 * for this controller and can often add instability unless tuned appropriately.
 * <p>
 * Uses feedforward based on {@code kA} and {@code kS} (velocity and static gain).
 * Uses momentum damping based off the robot's natural deceleration instead of {@code
 * kA} (acceleration gain).
 */
public class VelocityController extends PIDFController {
    public double naturalDeceleration;

    /**
     * Creates a simple velocity controller of a proportional and feedforward based
     * on kV and kS (velocity and static gain).
     * <p>
     * Does not use Integral or Derivative terms as they are usually unnecessary
     * for this controller and can often add instability unless tuned appropriately.
     * Uses momentum damping based off the robot's natural deceleration instead of {@code
     * kA} (acceleration gain).
     *
     * @param kP the proportional gain, how much power the robot reacts to error.
     *          Usually around 0.01.
     * @param kV the velocity gain, how much power the robot requires per
     *                  velocity. Should be around 1/maxVelocity or 1/65 ~= 0.015
     * @param kS the static gain, the minimum power required to start moving.
     *                Should be determined experimentally. Should be around 0.05.
     * @param naturalDeceleration the natural deceleration of your robot (in/s^2).
     *                            Should be positive. Used for momentum damping.
     */
    public VelocityController(double kP, double kV, double kS,
                              double naturalDeceleration) {
        super(kP, 0, 0, target -> target * kV + kS);
        this.naturalDeceleration = Validator.ensurePositiveSign(naturalDeceleration);
    }

    public double run(double targetVelocity, double currentVelocity,
                      double feedforwardVelocity) {
        double error = targetVelocity - currentVelocity;
        return runFromError(error, 0) + feedforward.compute(feedforwardVelocity);
    }
    
    public VelocityController setCoefficients(double kP, double kV, double kS, double naturalDeceleration) {
        this.kP = kP;
        this.feedforward = target -> target * kV + kS;
        this.naturalDeceleration = naturalDeceleration;
        return this;
    }
}
