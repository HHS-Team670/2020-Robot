package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.utils.motorcontroller.TalonSRXFactory;

/**
 * Superclass for any rotating subsystem using a TalonSRX, intakes for example
 * 
 * @author ctchen
 */
public abstract class TalonSRXRotatingSubsystem extends MustangSubsystemBase implements TunableSubsystem {
    protected static final int NO_SETPOINT = 99999;
    protected TalonSRX rotator;
    protected int setpoint;
    protected boolean timeout;
    protected double arbitraryFeedForwardConstant;
    protected int offsetFromEncoderZero;

    protected SensorCollection rotatorSensorCollection;

    public TalonSRXRotatingSubsystem(int deviceID, double arbitraryFeedForwardConstant, int forwardSoftLimit,
            int reverseSoftLimit, boolean timeout, int quadEncoderMin, int quadEncoderMax, int continuousCurrentLimit,
            int peakCurrentLimit, int offsetFromEncoderZero) {
        // For testing purposes
        this.rotator = TalonSRXFactory.buildFactoryTalonSRX(deviceID);
        this.rotatorSensorCollection = rotator.getSensorCollection();
        this.arbitraryFeedForwardConstant = arbitraryFeedForwardConstant;
        this.timeout = timeout;

        this.offsetFromEncoderZero = offsetFromEncoderZero;

        rotator.configFactoryDefault();

        rotator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        if (rotator != null) {
            this.rotator = rotator;
            this.rotatorSensorCollection = rotator.getSensorCollection();
            this.arbitraryFeedForwardConstant = arbitraryFeedForwardConstant;
            this.timeout = timeout;

            setpoint = TalonSRXRotatingSubsystem.NO_SETPOINT;

            int pulseWidthPos = getRotatorPulseWidth() & 4095;

            if (pulseWidthPos < quadEncoderMin) {
                pulseWidthPos += 4096;
            }
            if (pulseWidthPos > quadEncoderMax) {
                pulseWidthPos -= 4096;
            }

            rotatorSensorCollection.setQuadraturePosition(pulseWidthPos, 0);

            rotator.configContinuousCurrentLimit(continuousCurrentLimit);
            rotator.configPeakCurrentLimit(peakCurrentLimit);
            rotator.enableCurrentLimit(true);

            // These thresholds stop the motor when limit is reached
            rotator.configForwardSoftLimitThreshold(forwardSoftLimit);
            rotator.configReverseSoftLimitThreshold(reverseSoftLimit);

            // Enable Safety Measures
            rotator.configForwardSoftLimitEnable(true);
            rotator.configReverseSoftLimitEnable(true);

        }
    }

    /**
     * Gets the boolean to decide whether or not to pulse or stall the motor
     */
    public boolean getTimeout() {
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

    protected int getRotatorPulseWidth() {
        return getUnadjustedPulseWidth() - offsetFromEncoderZero;
    }

    protected int getUnadjustedPulseWidth() {
        return rotatorSensorCollection.getPulseWidthPosition();
    }

    public double getMotionMagicSetpoint() {
        return rotator.getClosedLoopTarget();
    }

    protected int getPositionTicks() {
        return rotator.getSelectedSensorPosition(0);
    }

    /**
     * Gets the multiplier for updating the arbitrary feed forward based on angle
     * and subsystem
     */
    // protected abstract double getArbitraryFeedForwardAngleMultiplier();

    /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public abstract void setMotionMagicSetpointAngle(double angle);

    public abstract double getAngleInDegrees();

}