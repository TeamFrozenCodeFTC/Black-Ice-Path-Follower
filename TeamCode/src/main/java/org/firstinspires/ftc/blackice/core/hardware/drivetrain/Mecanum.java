package org.firstinspires.ftc.blackice.core.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

class Mecanum extends Drivetrain {
    public final double strafingEffortMultiplier;

    public Mecanum(HardwareMap map, MecanumConfig config) {
        super(initMotors(map, config));
        this.strafingEffortMultiplier = config.maxForwardSpeed / config.maxStrafeSpeed;
    }

    public Vector adjustDirectionalEffort(Vector inputEffort) {
        return new Vector(
            inputEffort.getX(),
            inputEffort.getY() * this.strafingEffortMultiplier
        );
    }
    
    private WheelPowers getTranslationalPowers(Vector robotVector) {
        Vector adjustedVector = adjustDirectionalEffort(robotVector);
        
        double upRightDirection = -adjustedVector.getY() + adjustedVector.getX();
        double downLeftDirection = -adjustedVector.getY() - adjustedVector.getX();
        
        return new WheelPowers(
            upRightDirection, downLeftDirection,
            downLeftDirection, upRightDirection
        );
    }
    
    @Override
    public void followVector(Vector robotVector, double turnPower, boolean isTeleOp) {
        WheelPowers translationalPowers = getTranslationalPowers(robotVector);
        WheelPowers rotationalPowers = new WheelPowers(-turnPower, turnPower,
                                                       -turnPower, turnPower);
        
        applyPowers(translationalPowers
            .plus(rotationalPowers)
            .downscaleMaxTo(1)
        );
    }

    public void applyPowers(WheelPowers powers) {
        double[] motorPowers = powers.getPowers();
        for (int i = 0; i < motorPowers.length; i++) {
            motors[i].setPower(motorPowers[i]);
        }
    }
    
    private static DcMotorEx[] initMotors(HardwareMap map, MecanumConfig config) {
        DcMotorEx frontLeft = map.get(DcMotorEx.class, config.frontLeftName);
        DcMotorEx backLeft = map.get(DcMotorEx.class, config.backLeftName);
        DcMotorEx frontRight = map.get(DcMotorEx.class, config.frontRightName);
        DcMotorEx backRight = map.get(DcMotorEx.class, config.backRightName);
        
        frontLeft.setDirection(config.frontLeftDirection);
        backLeft.setDirection(config.backLeftDirection);
        frontRight.setDirection(config.frontRightDirection);
        backRight.setDirection(config.backRightDirection);

        return new DcMotorEx[]{ frontLeft, backLeft, frontRight, backRight };
    }
}
