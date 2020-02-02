package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.*;

public class Turret extends SparkMaxRotatingSubsystem {

    public final int TICKS_PER_REVOLUTION = 4096;
    private final double SOFT_LIMIT_IN_DEGREES = 160.0; // Default soft limit is in rotations. Will need to adjust this
    private static final double kFF = 0, kP = 0, kI = 0, kD = 0, kIz = 0; // TODO: find these values

    public Turret() {
        super(RobotMap.TURRET_ROTATOR, 0, kP, kI, kD, kFF, kIz, 1, -1, 5700, 2000, 0, 1500, 50, 1000, -1000, false, 30,
                0, 0);
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
        controller.setReference(getTicks(setpoint), ControlType.kPosition);
    }

    /**
     * 
     * @return encoder position in ticks
     */
    public double getEncoderPos() {

        return this.encoder.getPosition();// - referencePoint;
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
        return this.encoder;
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

    //TODO: define
    @Override
    public HealthState checkHealth() {
        if (rotator.getLastError() != null && rotator.getLastError() != CANError.kOk){
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