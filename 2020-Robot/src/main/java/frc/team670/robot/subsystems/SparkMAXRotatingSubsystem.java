package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;

import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SparkMAXRotatingSubsystem extends SubsystemBase implements TunableSubsystem{

    protected CANSparkMax rotator;
    protected CANEncoder encoder;
    protected CANPIDController controller;
    protected int offsetFromEncoderZero;
    protected double arbitraryFeedForwardConstant;
    protected double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public SparkMAXRotatingSubsystem(CANSparkMax rotatorSparkMax, int offsetFromEncoderZero) {
    
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

}