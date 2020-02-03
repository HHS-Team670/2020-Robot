package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.team670.robot.constants.RobotMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        //Slot for SmartMotion control
        public int getSlot() {
            return 0;
        }

        public double getP() {
            return SmartDashboard.getNumber("P Gain", 0.001);
        }

        public double getI() {
            return SmartDashboard.getNumber("I Gain", 0.001);
        }

        public double getD() {
            return SmartDashboard.getNumber("D Gain", 0.001);
        }

        public double getFF() {
            return SmartDashboard.getNumber("Feed Forward", 0.001);
        }

        public double getIz() {
            return SmartDashboard.getNumber("I Zone", 0.001);
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

    }

    public static final Config turretConfig = new Config();

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
        return ((getEncoderPos() / TICKS_PER_REVOLUTION) * 360);
        // verify if this is counts per revolution as the input of get ticks in degrees.
    }

    public void rotateToAngle(double setpoint) {
        this.rotator_controller.setReference(getTicks(setpoint), ControlType.kPosition);
    }

    /**
     * 
     * @return encoder position in ticks
     */
    public double getEncoderPos() {

        return this.rotator_encoder.getPosition();// - referencePoint;
    }

    /**
     * 
     * takes in double from -1 to 1 to set speed of turret motor
     * 
     * @param speed speed to set turret to
     */

    public void setTurretSpeed(double speed) {

        this.rotator.set(speed);

    }

    /**
     * 
     * @return double ratio of ticks to degrees. Can be multiplied by degrees to get
     *         ticks
     */
    public double getTicksPerDegree() {
        return TICKS_PER_REVOLUTION / 360;
    }

    /**
     * Converts degrees to ticks
     * 
     * @param degrees value to convert
     * @return
     */
    public double getTicks(double degrees) {
        return degrees * getTicksPerDegree();
    }

    /**
     * Converts ticks to degrees
     * 
     * @param ticks value to convert
     */
    public double getDegrees(double ticks) {
        return ticks / getTicksPerDegree();
    }

    /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(final double angle) {
        setpoint = (int) (angle * (getTicksPerDegree()));
    }

    /**
     * @return the encoder of the motor
     */
    public CANEncoder getEncoder() {
        return this.rotator_encoder;
    }

    /**
     * @return ???
     */
    @Override
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

    /**
     * Abstract method we have no clue what to do with
     */
    @Override
    public void moveByPercentOutput(double output) {
        // TODO Auto-generated method stub

    }

    // TODO: define
    @Override
    public HealthState checkHealth() {
        if (rotator.getLastError() != null && rotator.getLastError() != CANError.kOk) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    // Unused methods for potential later use.

    // /**
    // * motor health condition:
    // * indicates the current working capabilities,
    // * raises flag when turret motor is not functioning properly.
    // *
    // * @return
    // */
    // public boolean motorHealthConditions() {
    // // sparkControl

    // // This function is apparently depreciated
    // // double currentAmps = sparkControl.getOutputCurrent();

    // final double currentAmps = sparkControl.getSupplyCurrent(); // OR
    // // double currentAmps = sparkControl.getStatorCurrent();

    // final double outputVoltage = sparkControl.getMotorOutputVoltage();
    // final double busV = sparkControl.getBusVoltage();

    // /**
    // * double quadEncoderPos = sparkControl.getSelectedSensorPosition();
    // *
    // */

    // return false;
    // }

    /**
     * 
     * get encoder ticks
     * 
     * get power of motor, calculate potential ticks travelled.
     * 
     * talon
     * 
     * if encoder - potential = minorDifference. we are good else red flag
     * 
     * physical limit of turret
     * 
     * @param previousTicks comparing the previous ticks to current ticks lets us
     *                      know how much distance has been travelled over time,
     *                      helps in telling weather the encoder is working
     *                      especially when it stops when the motor is working.
     * @return flaggingFunctionality to return weather there is any problems with
     *         the subsystem.
     */
    // public boolean encoderHealthCondition(final double previousTicks) {
    // final double numberOfTicks = getEncoderPos();

    // // calculate potential travel in ticks
    // final double power = getSpeed();
    // return power != 0 && numberOfTicks != previousTicks;

    // }

}