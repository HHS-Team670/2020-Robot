package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Superclass for any rotating subsystem using a TalonSRX, intakes for example
 */
public abstract class RotatingSubsystem extends SubsystemBase implements TunableSubsystem {
    protected static final int NO_SETPOINT = 99999;
    protected TalonSRX rotator;
    protected int setpoint;
    protected boolean timeout;
    protected double arbitraryFeedForwardConstant;
    protected int offsetFromEncoderZero;

    protected SensorCollection rotatorSensorCollection;

    public RotatingSubsystem(CANSparkMax canSparkMax, double arbitraryFeedForwardConstant, int forwardSoftLimit, int reverseSoftLimit, boolean timeout, int quadEncoderMin, int quadEncoderMax, int continuousCurrentLimit, int peakCurrentLimit, int offsetFromEncoderZero) {
        // For testing purposes
        if (canSparkMax != null) {
            this.rotator = canSparkMax;
            this.rotatorSensorCollection = canSparkMax.getSensorCollection();
            this.arbitraryFeedForwardConstant = arbitraryFeedForwardConstant;
            this.timeout = timeout;

            this.offsetFromEncoderZero = offsetFromEncoderZero;

            canSparkMax.configFactoryDefault();

            canSparkMax.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

            if (canSparkMax != null) {
                this.rotator = canSparkMax;
                this.rotatorSensorCollection = canSparkMax.getSensorCollection();
                this.arbitraryFeedForwardConstant = arbitraryFeedForwardConstant;
                this.timeout = timeout;

                setpoint = RotatingSubsystem.NO_SETPOINT;

                int pulseWidthPos = getRotatorPulseWidth() & 4095;

            if (pulseWidthPos < quadEncoderMin) {
              pulseWidthPos += 4096;
            }
            if (pulseWidthPos > quadEncoderMax) {
                pulseWidthPos -= 4096;
            }

            rotatorSensorCollection.setQuadraturePosition(pulseWidthPos, 0);

            canSparkMax.configContinuousCurrentLimit(continuousCurrentLimit);
            canSparkMax.configPeakCurrentLimit(peakCurrentLimit);
            canSparkMax.enableCurrentLimit(true);

            // These thresholds stop the motor when limit is reached
            canSparkMax.configForwardSoftLimitThreshold(forwardSoftLimit);
            canSparkMax.configReverseSoftLimitThreshold(reverseSoftLimit);

            // Enable Safety Measures
            canSparkMax.configForwardSoftLimitEnable(true);
            canSparkMax.configReverseSoftLimitEnable(true);
            
            }
        }
    }

    /**
     * Gets the boolean to decide whether or not to pulse or stall the motor
     */
    public boolean getTimeout(){
        return timeout;
    }

    public void enableCoastMode() {
        rotator.setNeutralMode(NeutralMode.Coast);
    }

    public void enableBrakeMode() {
        rotator.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Puts the main talon in percent output mode
     */
    public synchronized void stop() {
        clearSetpoint();
        rotator.set(ControlMode.PercentOutput, 0);
        System.out.println("Put in Percent Output");
    }

    /**
     * Removes the setpoint for the talon on this subsystem
     */
    public void clearSetpoint() {
        setpoint = NO_SETPOINT;
    }


    /**
     * Rotates the talon at a certain percent output
     */
    public void moveByPercentOutput(double output) {
        rotator.set(ControlMode.PercentOutput, output);
    }

    /**
     * Updates the arbitrary feed forward on this subsystem
     */
    public synchronized void updateArbitraryFeedForward(){
        if(setpoint != NO_SETPOINT) {
            double value = getArbitraryFeedForwardAngleMultiplier() * arbitraryFeedForwardConstant;
            rotator.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, value);
          }
    }

    protected int getRotatorPulseWidth(){
        return getUnadjustedPulseWidth() - offsetFromEncoderZero;
    }

    protected int getUnadjustedPulseWidth() {
        return rotatorSensorCollection.getPulseWidthPosition();
    }

    public double getMotionMagicSetpoint(){
        return rotator.getClosedLoopTarget();
    }

    protected int getPositionTicks(){
        return rotator.getSelectedSensorPosition(0);
    }

    /**
     * Gets the multiplier for updating the arbitrary feed forward based on angle and subsystem
     */
    protected abstract double getArbitraryFeedForwardAngleMultiplier();

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public abstract void setMotionMagicSetpointAngle(double angle);

    public abstract double getAngleInDegrees();


}