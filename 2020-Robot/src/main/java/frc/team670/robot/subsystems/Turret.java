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


//
public class Turret extends RotatingSubsystem
{

    
    private TalonSRX talonControl;
    private int countsPerRevolution; //ticks per revolution

    public Turret(/*int mainCANId,*/ int cpr)
    {

        super(new TalonSRX(RobotMap.TALON_TURRET), 
0, 180, 0, true, 0, 0, 0, 0, 0); //TODO Complete this
        this.talonControl = super.rotator;this.countsPerRevolution = cpr;
        
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

    //angular speed, not linear speed
    public double getSpeed() {
//getTicksInDegrees
        //angle travelled / time

        

        return 0;//getEncoder().getPulseWidthPosition();
    }

    public int getEncoderPos() {
        
        return getEncoder().getPulseWidthPosition();
    }


    public boolean motorHealthConditions() {
        // talonControl 

        double currentAmps = talonControl.getOutputCurrent();
        double outputVoltage = talonControl.getMotorOutputVoltage();
        double busV = talonControl.getBusVoltage();
        

        /**
         * double quadEncoderPos = talonControl.getSelectedSensorPosition();
         * 
         */


    }







    /**
     * 
     * take in degrees, 
     * 
     * move that turret the amount of degrees, 
     *      -   Convention:     positive is clockwise. 
     * 
     * 
     */
    public void rotateTurret(double degrees) {
        talonControl.set(ControlMode.Position, degrees);
         
        

         

     }

    public boolean isWorking(double previousTicks) {
        double numberOfTicks = getEncoderPos();

        // calculate potential travel in ticks

        double power = getSpeed();
        return power != 0 && numberOfTicks != previousTicks;
        

    
        /**
         * get encoder ticks
         * 
         * get power of motor, 
         *      calculate potential ticks travelled. 
         * 
         * talon
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
        return countsPerRevolution/360.0;
   return countsPerRevolution/360.0;
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(double angle)
    {
        setpoint = (int)(angle*(getTicksInDegrees()));
   setpoint = (int)(angle*(getTicksInDegrees()));
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(double angle)
    {
        setpoint = (int)(angle*(getTicksInDegrees()));
    }

    public SensorCollection getEncoder() //   TODO This is not CANEncoder, this has to be changed later. 
    {
        
    }