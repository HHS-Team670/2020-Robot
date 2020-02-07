package frc.team670.robot.subsystems;

import com.revrobotics.CANError;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

public class Turret extends SparkMaxRotatingSubsystem {

    // TODO: Set these values
    public static final int SOFT_MINIMUM_DEGREES = -30;
    public static final int SOFT_MAXIMUM_DEGREES = 210;

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
            return 0.001;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
        }

        public double getFF() {
            return 0;
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
            return 10;
        }

        public double getMinVelocity() {
            return 0;
        }

        public double getMaxAcceleration() {
            return 1.5;
        }

        public double getAllowedError() {
            return 2;
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
    private final int DEGREES_PER_MOTOR_ROTATION = 36;

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

    // TODO: define
    @Override
    public HealthState checkHealth() {
        if (rotator.getLastError() != null && rotator.getLastError() != CANError.kOk) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void moveByPercentOutput(double output) {
        // TODO Auto-generated method stub

    }

    @Override
    public double getCurrentAngleInDegrees() {
        return getUnadjustedPosition() * DEGREES_PER_MOTOR_ROTATION;
    }

    @Override
    protected double getMotorRotationsFromAngle(double angle) {
        // TODO Auto-generated method stub
        return 0;
    }

}