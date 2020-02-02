package frc.team670.robot.subsystems;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANPIDController;

/**
 * Superclass for any rotating subsystem which uses a SparkMax to control the rotator.
 * 
 * @author ctychen
 */
public abstract class SparkMaxRotatingSubsystem extends MustangSubsystemBase implements TunableSubsystem {

    protected CANSparkMax rotator;
    protected CANEncoder encoder;
    protected CANPIDController controller;
    protected int offsetFromEncoderZero;
    protected static final double NO_SETPOINT = Double.NaN; // TODO: why 99999?
    protected double setpoint;
    protected boolean timeout;
    protected double kP, kI, kD, kFF, kIz, MAX_OUTPUT, MIN_OUTPUT, MAX_RPM;
    protected double MAX_VEL, MIN_VEL, MAX_ACC, ALLOWED_ERR;
    protected int SMARTMOTION_SLOT;

    public SparkMaxRotatingSubsystem(int deviceID, int slot, double kP, double kI, double kD, 
            double kFF, double kIz, double KMaxOutput,
            double kMinOutput, double kMaxRPM, double kMaxVel, 
            double kMinVel, double kMaxAcc, double kAllowedErr,
            int forwardSoftLimit, int reverseSoftLimit, boolean timeout, 
            int continuousCurrentLimit,
            int peakCurrentLimit, int offsetFromEncoderZero) {
        this.rotator = SparkMAXFactory.buildFactorySparkMAX(deviceID);
        this.encoder = rotator.getEncoder();
        this.offsetFromEncoderZero = offsetFromEncoderZero;
        this.timeout = timeout;

        // PID coefficients
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kIz = kIz;
        this.kFF = kFF;
        this.MAX_OUTPUT = KMaxOutput;
        this.MIN_OUTPUT = kMinOutput;
        this.MAX_RPM = kMaxRPM;

        // Smart Motion Coefficients
        this.MAX_VEL = kMaxVel; // rpm
        this.MAX_ACC = kMaxAcc;
        this.ALLOWED_ERR = kAllowedErr;

        // set PID coefficients
        this.controller.setP(kP);
        this.controller.setI(kI);
        this.controller.setD(kD);
        this.controller.setIZone(kIz);
        this.controller.setFF(kFF);
        this.controller.setOutputRange(this.MIN_OUTPUT, this.MAX_OUTPUT);

        this.SMARTMOTION_SLOT = slot;
        controller.setSmartMotionMaxVelocity(this.MAX_VEL, this.SMARTMOTION_SLOT);
        controller.setSmartMotionMinOutputVelocity(this.MIN_VEL, this.SMARTMOTION_SLOT);
        controller.setSmartMotionMaxAccel(this.MAX_ACC, this.SMARTMOTION_SLOT);
        controller.setSmartMotionAllowedClosedLoopError(this.ALLOWED_ERR, this.SMARTMOTION_SLOT);

        rotator.setSmartCurrentLimit(peakCurrentLimit, continuousCurrentLimit);

        rotator.setSoftLimit(SoftLimitDirection.kForward, forwardSoftLimit);
        rotator.setSoftLimit(SoftLimitDirection.kReverse, reverseSoftLimit);

        rotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        rotator.enableSoftLimit(SoftLimitDirection.kReverse, true);

    }

    protected double getUnadjustedPosition() {
        return this.encoder.getPosition();
    }

    public void setSmartMotionTarget(double setpoint) {
        controller.setReference(setpoint, ControlType.kSmartMotion);
    }

    public abstract double getAngleInDegrees();

    public void enableCoastMode() {
        rotator.setIdleMode(IdleMode.kCoast);
    }

    public void enableBrakeMode() {
        rotator.setIdleMode(IdleMode.kBrake);
    }

    public synchronized void stop() {
        clearSetpoint();
        rotator.set(0);
    }

    public void clearSetpoint() {
        setpoint = NO_SETPOINT; // TODO: is NO_SETPOINT a good value?
    }

    public CANSparkMax getRotator() {
        return this.rotator;
    }

    public CANPIDController getController() {
        return this.controller;
    }
}