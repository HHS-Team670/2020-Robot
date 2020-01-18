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
    private TalonSRX talonControl;
    private int countsPerRevolution; //ticks per revolution

    public Turret(/*int mainCANId,*/ int cpr)
    {
        // NOTE ignore error, will deal with this later. 
        super(new TalonSRX(RobotMap.TALON_TURRET), 0, 180, 0, true, 0, 0, 0, 0, 0);         //TODO Complete this
        this.talonControl = super.rotator;this.countsPerRevolution = cpr;
        
    }

    public double getAngleInDegrees()
    {
        //there are 4096 ticks in a circle, so one degree is 11 17/45 ticks.
        return super.getPositionTicks() / (getTicksInDegrees(countsPerRevolution));             // verify if this is counts per revolution as the input of get ticks in degrees. 
    }

    /**
     * TODO: implement this method
     */
    protected double getArbitraryFeedForwardAngleMultiplier()
    {
        return 0.0;
    }

    /**
     * angular speed, not linear speed
     * 
     * TODO Maybe change this, pretty obvious?!!
     * 
     * @return
     */
    public double getSpeed() {
        //getTicksInDegrees
        //angle travelled / time

        return 0;           //getEncoder().getPulseWidthPosition();
    }

    public int getEncoderPos() {
        return getEncoder().getPulseWidthPosition();
    }

    /**
     * motor health condition: 
     *          indicates the current working capabilities, 
     *                  raises flag when turret motor is not functioning properly. 
     * 
     * @return
     */
    public boolean motorHealthConditions() {
        // talonControl 

        //  This function is apparently depreciated
//        double currentAmps = talonControl.getOutputCurrent();

        double currentAmps = talonControl.getSupplyCurrent();   // OR
//        double currentAmps = talonControl.getStatorCurrent();

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
     * @param degrees
     *      possibly amount of degrees needed to turn clockwise, 
     *          this could also rather be in radians or can be some set ticks to rotate the motor. 
     */
    public void rotateTurret(double degrees) {
        talonControl.set(ControlMode.Position, degrees);
         
        // TODO research this and make changes, this section is not complete. 
     }

    /**
     * 
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
     * @param previousTicks
     *      comparing the previous ticks to current ticks lets us know how much distance has been travelled over time, 
     *                  helps in telling weather the encoder is working especially when it stops when the motor is working. 
     * @return flaggingFunctionality
     *      to return weather there is any problems with the subsystem. 
     */
    public boolean encoderHealthCondition(double previousTicks) {
        double numberOfTicks = getEncoderPos();

        // calculate potential travel in ticks
        double power = getSpeed();
        return power != 0 && numberOfTicks != previousTicks;
    
    }


    public double getTicksInDegrees(int numberOfTicks) {
        return (numberOfTicks/360);                         // Counts per revolution is a int type, 360.0 to 360. 
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(double angle)
    {
        setpoint = (int)(angle*(getTicksInDegrees(countsPerRevolution)));
    }

    public SensorCollection getEncoder() //   TODO This is not CANEncoder, this has to be changed later. 
    {
        return null;                    // TODO FIX THIS
    }