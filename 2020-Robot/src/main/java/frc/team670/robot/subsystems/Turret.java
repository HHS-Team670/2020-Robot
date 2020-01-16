package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends RotatingSubsystem
{
    private CANEncoder encoder;

    public Turret(TalonSRX rotator, int mainCANId, CANEncoder encoder)
    {
        super(rotator, 0, 180, 0, true, 0, 0, 0, 0, 0); //TODO Complete this
        this.encoder = encoder;
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

    public double getTicksInDegrees()
    {
        return getEncoder().getCountsPerRevolution();
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(double angle)
    {
        setpoint = (int)(angle*(11.0 + 17.0/45.0));
    }

    public CANEncoder getEncoder()
    {
        return encoder;
    }

}