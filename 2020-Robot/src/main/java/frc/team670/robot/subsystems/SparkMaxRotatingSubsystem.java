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
 * Superclass for any rotating subsystem which uses a SparkMax to control the
 * rotator.
 * 
 * @author ctychen
 */
public abstract class SparkMaxRotatingSubsystem extends MustangSubsystemBase implements TunableSubsystem {

    protected CANSparkMax rotator;
    protected CANEncoder rotator_encoder;
    protected CANPIDController rotator_controller;
    protected int offsetFromEncoderZero;
    protected static final double NO_SETPOINT = Double.NaN;
    protected double setpoint;
    protected double kP, kI, kD, kFF, kIz, MAX_OUTPUT, MIN_OUTPUT, MAX_RPM;
    protected double MAX_VEL, MIN_VEL, MAX_ACC, ALLOWED_ERR;
    protected int SMARTMOTION_SLOT;

    /**
     * Configuration for this RotatingSubsystem's properties. 
     * Use this to keep track of PID and SmartMotion constants
     */
    public static abstract class Config {

        public abstract int getDeviceID();

        public abstract int getSlot();

        public abstract double getP();

        public abstract double getI();

        public abstract double getD();

        public abstract double getFF();

        public abstract double getIz();

        public abstract double getMaxOutput();

        public abstract double getMinOutput();

        public abstract double getMaxRPM();

        public abstract double getMaxVelocity();

        public abstract double getMinVelocity();

        public abstract double getMaxAcceleration();

        public abstract double getAllowedError();

        public abstract float getForwardSoftLimit();

        public abstract float getReverseSoftLimit();

        public abstract int getContinuousCurrent();

        public abstract int getPeakCurrent();

        public abstract int getOffsetFromEncoderZero();

    }

    public SparkMaxRotatingSubsystem(Config config) {
        this.rotator = SparkMAXFactory.buildFactorySparkMAX(config.getDeviceID());
        this.rotator_controller = rotator.getPIDController();
        this.offsetFromEncoderZero = config.getOffsetFromEncoderZero();

        // PID coefficients
        this.kP = config.getP();
        this.kI = config.getI();
        this.kD = config.getD();
        this.kIz = config.getIz();
        this.kFF = config.getFF();
        this.MAX_OUTPUT = config.getMaxOutput();
        this.MIN_OUTPUT = config.getMinOutput();
        this.MAX_RPM = config.getMaxRPM();

        // Smart Motion Coefficients
        this.MAX_VEL = config.getMaxVelocity(); // rpm
        this.MAX_ACC = config.getMaxAcceleration();
        this.ALLOWED_ERR = config.getAllowedError();

        // set PID coefficients
        this.rotator_controller.setP(kP);
        this.rotator_controller.setI(kI);
        this.rotator_controller.setD(kD);
        this.rotator_controller.setIZone(kIz);
        this.rotator_controller.setFF(kFF);
        this.rotator_controller.setOutputRange(this.MIN_OUTPUT, this.MAX_OUTPUT);

        this.SMARTMOTION_SLOT = config.getSlot();
        rotator_controller.setSmartMotionMaxVelocity(this.MAX_VEL, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionMinOutputVelocity(this.MIN_VEL, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionMaxAccel(this.MAX_ACC, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionAllowedClosedLoopError(this.ALLOWED_ERR, this.SMARTMOTION_SLOT);

        rotator.setSmartCurrentLimit(config.getPeakCurrent(), config.getContinuousCurrent());

        rotator.setSoftLimit(SoftLimitDirection.kForward, config.getForwardSoftLimit());
        rotator.setSoftLimit(SoftLimitDirection.kReverse, config.getReverseSoftLimit());

        rotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        rotator.enableSoftLimit(SoftLimitDirection.kReverse, true);

    }

    public double getUnadjustedPosition() {
        return this.rotator_encoder.getPosition();
    }

    public void setSmartMotionTarget(double setpoint) {
        rotator_controller.setReference(setpoint, ControlType.kSmartMotion);
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

    public CANEncoder getRotatorEncoder() {
        return this.rotator_encoder;
    }

    public CANPIDController getRotatorController() {
        return this.rotator_controller;
    }
}