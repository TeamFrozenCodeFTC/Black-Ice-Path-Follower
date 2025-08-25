package org.firstinspires.ftc.blackice.core.control;

/**
 * A PD controller with quadratic damping **quadratic-damped PIDs** to model the non-linear effects of friction and momentum.but more empirical. Also has basically a
 * Derivative of
 * Derivative term or velocity^2
 * term to help stop more smoothly because EMF braking is not perfectly quadratic or linear.
 * Also is predictive because D term compensates for predictedPosition = position + velocity * deltaTime.
 */
public class QuadraticDampedPIDController extends PIDController {
    private double kBraking;
    private double kFriction;
    
    public QuadraticDampedPIDController(double kP, double kLinearBraking,
                                        double kQuadraticFriction) {
        super(kP, 0, 0);
        this.kBraking = kLinearBraking;
        this.kFriction = kQuadraticFriction;
    }
    
    @Override
    public double runFromError(double error, double derivative) {
        double velocity = -derivative;
        double predictedBrakingDisplacement = velocity * Math.abs(velocity) * kFriction + velocity *
            kBraking;
        return super.runFromError(
            error - predictedBrakingDisplacement,
            derivative
        );
    }
    
    public QuadraticDampedPIDController setCoefficients(double kP, double kLinearBraking,
                                          double kQuadraticFriction) {
        this.kP = kP;
        this.kBraking = kLinearBraking;
        this.kFriction = kQuadraticFriction;
        return this;
    }
}
