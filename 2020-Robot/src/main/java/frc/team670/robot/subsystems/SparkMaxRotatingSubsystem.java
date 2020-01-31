package frc.team670.robot.subsystems;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANPIDController;

public abstract class SparkMaxRotatingSubsystem extends MustangSubsystemBase implements TunableSubsystem {

    protected CANSparkMax rotator;
    protected CANEncoder encoder;
    protected CANPIDController controller;
    protected int offsetFromEncoderZero;
    protected static final int NO_SETPOINT = 99999; // TODO: why 99999?
    protected int setpoint;
    protected boolean timeout;

    public SparkMaxRotatingSubsystem(int deviceID, double kP, double kI, double kD, double kFF, int forwardSoftLimit,
            int reverseSoftLimit, boolean timeout, int continuousCurrentLimit, int peakCurrentLimit,
            int offsetFromEncoderZero) {
        this.rotator = SparkMAXFactory.buildFactorySparkMAX(deviceID);// TODO: let's ask claire what this is supposed to
                                                                      // do
        this.encoder = rotator.getEncoder();
        this.offsetFromEncoderZero = offsetFromEncoderZero;
        this.timeout = timeout;

        controller = this.rotator.getPIDController();
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setFF(kFF);

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

}