package frc.team670.robot.subsystems;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.motorcontroller.MotorConfig;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;

import com.revrobotics.CANEncoder;
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

    protected SparkMAXLite rotator;
    protected CANEncoder rotator_encoder;
    protected CANPIDController rotator_controller;
    protected static final double NO_SETPOINT = Double.NaN;
    protected double setpoint;
    protected double kP, kI, kD, kFF, kIz, MAX_OUTPUT, MIN_OUTPUT;
    protected double MAX_VEL, MIN_VEL, MAX_ACC, ALLOWED_ERR;
    protected int SMARTMOTION_SLOT;
    protected double ROTATOR_GEAR_RATIO;

    /**
     * Configuration for this RotatingSubsystem's properties. Use this to keep track
     * of PID and SmartMotion constants
     */
    public static abstract class Config {

        public abstract int getDeviceID();

        public abstract int getSlot();

        public abstract MotorConfig.Motor_Type getMotorType();

        public abstract IdleMode setRotatorIdleMode();

        public abstract double getRotatorGearRatio();

        public abstract double getP();

        public abstract double getI();

        public abstract double getD();

        public abstract double getFF();

        public abstract double getIz();

        public abstract double getMaxOutput();

        public abstract double getMinOutput();

        public abstract double getMaxVelocity();

        public abstract double getMinVelocity();

        public abstract double getMaxAcceleration();

        public abstract double getAllowedError();

        /**
         * @return An array of soft limits, in rotations, for this system:
         *         [forwardLimit, reverseLimit].
         * @return null if this system does not have soft limits.
         */
        public abstract float[] setSoftLimits();

        public abstract int getContinuousCurrent();

        public abstract int getPeakCurrent();
    }

    public SparkMaxRotatingSubsystem(Config config) {
        this.rotator = SparkMAXFactory.buildFactorySparkMAX(config.getDeviceID(), config.getMotorType());
        this.rotator_encoder = rotator.getEncoder();
        this.rotator_controller = rotator.getPIDController();

        this.ROTATOR_GEAR_RATIO = config.getRotatorGearRatio();

        // PID coefficients
        this.kP = config.getP();
        this.kI = config.getI();
        this.kD = config.getD();
        this.kIz = config.getIz();
        this.kFF = config.getFF();
        this.MAX_OUTPUT = config.getMaxOutput();
        this.MIN_OUTPUT = config.getMinOutput();

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

        if (config.setSoftLimits() == null || config.setSoftLimits().length > 2) {
            rotator.enableSoftLimit(SoftLimitDirection.kForward, false);
            rotator.enableSoftLimit(SoftLimitDirection.kReverse, false);
        } else {
            rotator.setSoftLimit(SoftLimitDirection.kForward, config.setSoftLimits()[0]);
            rotator.setSoftLimit(SoftLimitDirection.kReverse, config.setSoftLimits()[1]);
        }

    }

    public double getUnadjustedPosition() {
        return this.rotator_encoder.getPosition();
    }

    protected void setSmartMotionTarget(double setpoint) {
        rotator_controller.setReference(setpoint, ControlType.kSmartMotion);
        this.setpoint = setpoint;
    }

    public void setTargetAngleInDegrees(double angle) {
        setSmartMotionTarget(getMotorRotationsFromAngle(angle));
    }

    protected double getMotorRotationsFromAngle(double angle) {
        return (angle / 360) * this.ROTATOR_GEAR_RATIO;
    }

    public abstract double getCurrentAngleInDegrees();

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
        setpoint = NO_SETPOINT;
    }

    public SparkMAXLite getRotator() {
        return this.rotator;
    }

    public CANEncoder getRotatorEncoder() {
        return this.rotator_encoder;
    }

    public CANPIDController getRotatorController() {
        return this.rotator_controller;
    }
}