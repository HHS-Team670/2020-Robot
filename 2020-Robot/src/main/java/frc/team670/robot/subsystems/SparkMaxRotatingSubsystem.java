package frc.team670.robot.subsystems;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANPIDController;

/** 
 * Superclass for any subsystem using a SparkMax controller.
 * 
 * @author ctychen
*/
public abstract class SparkMaxRotatingSubsystem extends MustangSubsystemBase implements TunableSubsystem{

    protected CANSparkMax rotator;
    protected CANEncoder encoder;
    protected CANPIDController controller;
    protected int offsetFromEncoderZero;
    protected static final int NO_SETPOINT = 99999;
    protected int setpoint;

    public SparkMaxRotatingSubsystem(CANSparkMax rotatorSparkMax, int offsetFromEncoderZero) {
    
        if (rotatorSparkMax != null) {
            this.rotator = rotatorSparkMax;
            this.encoder = rotatorSparkMax.getEncoder();
            this.offsetFromEncoderZero = offsetFromEncoderZero;
            SparkMAXFactory.buildFactorySparkMAX(rotatorSparkMax.getDeviceId());
        }
    }

    protected int getUnadjustedPosition() {
        return (int)(this.encoder.getPosition());
    }

    public void setSmartMotionTarget(double setpoint){
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
        setpoint = NO_SETPOINT;
    }

}