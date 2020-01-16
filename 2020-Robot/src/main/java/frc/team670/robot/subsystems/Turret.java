package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANAnalog.AnalogMode;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.RobotContainer;

public class Turret extends RotatingSubsystem
{
    private CANSparkMax speedController;
    private int countsPerRevolution;

    public Turret(TalonSRX rotator, int mainCANId, CANSparkMax motor, int cpm)
    {
        super(rotator, 0, 180, 0, true, 0, 0, 0, 0, 0); //TODO Complete this
        this.speedController = motor;
        this.countsPerRevolution = cpm;
    }

    
    public double getAngleInDegrees()
    {
        //there are 4096 ticks in a circle, so one degree is 11 17/45 ticks.
        return super.getPositionTicks() / (getTicksInDegrees());
    }

    /**
     * TODO: implement this method
     */
    protected double getArbitraryFeedForwardAngleMultiplier()
    {
        return 0.0;
    }

    /**
     * TODO: Maybe change it
     */
    public double getSpeed() {
        return speedController.get();
    }


    public boolean isWorking(double previousTicks) {
        double numberOfTicks = getEncoder().getPosition();

        // calculate potential travel in ticks

        double power = getSpeed();
        return power != 0 && numberOfTicks != previousTicks;
        

        /**
         * get encoder ticks
         * 
         * get power of motor, 
         *      calculate potential ticks travelled. 
         * 
         * if encoder - potential = minorDifference. 
         *      we are good
         * else
         *      red flag
         * 
         */
        
    }


    public double getTicksInDegrees()
    {
        return getEncoder().getCountsPerRevolution()/360.0;
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(double angle)
    {
        setpoint = (int)(angle*(getTicksInDegrees()));
    }

    public CANEncoder getEncoder()
    {
        return speedController.getEncoder(EncoderType.kHallSensor, countsPerRevolution);
    }

}