package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

public class Turret extends SparkMaxRotatingSubsystem {

    public final int TICKS_PER_REVOLUTION = 4096;
    private final double SOFT_LIMIT_IN_DEGREES = 160.0; // Default soft limit is in rotations. Will need to adjust this

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

        public double getMaxRPM() {
            return 5700;
        }

        public double getMaxVelocity() {
            return 2000;
        }

        public double getMinVelocity() {
            return 0;
        }

        public double getMaxAcceleration() {
            return 1500;
        }

        public double getAllowedError() {
            return 50;
        }

        public float getForwardSoftLimit() {
            return 1000;
        }

        public float getReverseSoftLimit() {
            return -1000;
        }

        public int getContinuousCurrent() {
            return 30;
        }

        public int getPeakCurrent() {
            return 0;
        }

        public int getOffsetFromEncoderZero() {
            return 0;
        }

        @Override
        public Motor_Type getMotorType() {
            return Motor_Type.NEO_550;
        }

    }

    public static final Config turretConfig = new Config();
    private final int DEGREES_PER_MOTOR_ROTATION = 36;

    public Turret() {
        super(turretConfig);
    }

    /**
     * 
     * @return the position of the turret in degrees
     */
    public double getAngleInDegrees() {
        // define degrees

        // there are 4096 ticks in a circle, so one degree is 11 17/45 ticks.
        // return ((getEncoderPos() / TICKS_PER_REVOLUTION) * 360);
        return getUnadjustedPosition() * DEGREES_PER_MOTOR_ROTATION;
        // verify if this is counts per revolution as the input of get ticks in degrees.
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
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    protected double getMotorRotationsFromAngle(double angle) {
        // TODO Auto-generated method stub
        return 0;
    }

}