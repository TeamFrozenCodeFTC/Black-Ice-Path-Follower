package org.firstinspires.ftc.blackice.core.control;

/**
 * PIDController implements a basic Proportional-Integral-Derivative control algorithm.
 * It calculates an output value based on the error between a target and current value,
 * with optional integral and derivative terms.
 */
public class PIDController {
    double kP, kI, kD;
    double previousError, integralSum;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double run(double target, double current, double deltaTime) {
        double error = target - current;
        double errorRate = kD != 0 ? (error - previousError) / deltaTime : 0;
        return runFromError(error, errorRate);
    }

    public double runFromError(double error, double derivative) {
        double output = kP * error;

        if (kI != 0) {
            integralSum += error;
            output += kI * integralSum;
        }
        if (kD != 0) output += kD * derivative;

        previousError = error;
        return output;
    }

    public void reset() {
        previousError = 0;
        integralSum = 0;
    }
    
    public PIDController setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }
}
