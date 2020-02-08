package frc.team670.robot.subsystems;

import com.revrobotics.CANError;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

public class Turret extends SparkMaxRotatingSubsystem {

    // TODO: Set these values. Keeping it small right now for testing.
    public static final int SOFT_MINIMUM_DEGREES = -30;
    public static final int SOFT_MAXIMUM_DEGREES = 30;

    /**
     * Constants for the turret go here; this includes PID and SmartMotion values.
     * TODO: find what these values should be, because everything in here is a
     * placeholder
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.TURRET_ROTATOR;
        }

        // Slot for SmartMotion control
        public int getSlot() {
            return 0;
        }

        public double getP() {
            return 0.00005;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
        }

        public double getFF() {
            return 0.0012;
        }

        public double getIz() {
            return 0;
        }

        public double getMaxOutput() {
            return 1;
        }

        public double getMinOutput() {
            return -1;
        }

        public double getMaxVelocity() {
            return 200; //TODO: probably needs to be adjusted
        }

        public double getMinVelocity() {
            return 0;
        }

        public double getMaxAcceleration() {
            return 200; //TODO: probably needs to be adjusted
        }

        public double getAllowedError() {
            // equivalent of 2 degrees, in rotations
            return (2 / 360) * this.getRotatorGearRatio();
        }

        public float getForwardSoftLimit() {
            return (float) (SOFT_MAXIMUM_DEGREES * getRotatorGearRatio());
        }

        public float getReverseSoftLimit() {
            return (float) (SOFT_MINIMUM_DEGREES * getRotatorGearRatio());
        }

        public int getContinuousCurrent() {
            return 3; // Placeholder
        }

        public int getPeakCurrent() {
            return 6; // Placeholder
        }

        public Motor_Type getMotorType() {
            return Motor_Type.NEO_550;
        }

        public double getRotatorGearRatio() {
            return 71.166;
        }

    }

    public static final Config turretConfig = new Config();
    private final double DEGREES_PER_MOTOR_ROTATION = 360/turretConfig.getRotatorGearRatio();

    public Turret() {
        super(turretConfig);
    }

    /**
     * 
     * takes in double from -1 to 1 to set speed of turret motor
     * 
     * @pre speed has to be greater than or equal to negative one and less than or
     *      equal to one.
     * @param speed speed to set turret to
     */

    public void setTurretSpeed(double speed) {
        this.rotator.set(speed);
    }

    @Override
    public HealthState checkHealth() {
        if (isSparkMaxHealthy(rotator)) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void setTargetAngleInDegrees(double targetAngle) {
        if (targetAngle > SOFT_MAXIMUM_DEGREES || targetAngle < SOFT_MINIMUM_DEGREES) {
            throw new IllegalArgumentException(
                    "Invalid angle: must be within range " + SOFT_MINIMUM_DEGREES + " and " + SOFT_MAXIMUM_DEGREES);
        } else {
            super.setTargetAngleInDegrees(targetAngle);
        }
    }

    @Override
    public double getCurrentAngleInDegrees() {
        return getUnadjustedPosition() * DEGREES_PER_MOTOR_ROTATION;
    }

    @Override
    public void moveByPercentOutput(double output) {
        // TODO Auto-generated method stub

    }

}