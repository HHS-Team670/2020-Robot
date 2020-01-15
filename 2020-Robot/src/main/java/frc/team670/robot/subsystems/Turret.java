package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends RotatingSubsystem
{

    public Turret(TalonSRX rotatorTalon, double arbitraryFeedForwardConstant, int forwardSoftLimit, int reverseSoftLimit, boolean timeout, int quadEncoderMin, int quadEncoderMax, int continuousCurrentLimit, int peakCurrentLimit, int offsetFromEncoderZero)
    {
        super(rotatorTalon,  arbitraryFeedForwardConstant,  forwardSoftLimit,  reverseSoftLimit,  timeout,  quadEncoderMin,  quadEncoderMax,  continuousCurrentLimit,  peakCurrentLimit,  offsetFromEncoderZero);
    }

    
    public double getAngleInDegrees()
    {
        //there are 4096 ticks in a circle, so one degree is 11 17/45 ticks.
        return super.getPositionTicks() / (11.0 + (17.0/45.0));
    }

    /**
     * TO DO: implement this method
     */
    protected double getArbitraryFeedForwardAngleMultiplier()
    {
        return 0.0;
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(double angle)
    {
        setpoint = (int)(angle*(11.0 + 17.0/45.0));
    }


}