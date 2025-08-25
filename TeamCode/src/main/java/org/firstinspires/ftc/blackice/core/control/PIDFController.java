package org.firstinspires.ftc.blackice.core.control;


/**
 * Represents a PIDF (Proportional-Integral-Derivative-Feedforward) controller.
 */
public class PIDFController extends PIDController {
    @FunctionalInterface
    public interface Feedforward {
        double compute(double target);
    }
    
    public Feedforward feedforward;

    public PIDFController(double kP, double kI, double kD, double feedforwardGain) {
        this(kP, kI, kD, target -> target * feedforwardGain);
    }

    public PIDFController(double kP, double kI, double kD, Feedforward feedforward) {
        super(kP, kI, kD);
        this.feedforward = feedforward;
    }
    
    @Override
    public double run(double target, double current, double deltaTime) {
        return super.run(target, current, deltaTime) + feedforward.compute(target);
    }
    
    public PIDFController setCoefficients(double kP, double kI, double kD, Feedforward feedforward) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.feedforward = feedforward;
        return this;
    }
}
