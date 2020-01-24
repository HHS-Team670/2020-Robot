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
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.*;

public class Turret extends RotatingSubsystem
{
    private final TalonSRX talonControl;
    private final int TICKS_PER_REVOLUTION = 4096; //ticks per revolution
    private final int TURRET_LIMIT = (int)((TICKS_PER_REVOLUTION * 160.0)/360.0);                  // assuming the turret's soft limit will be 1820 ticks = 160 degrees both ways from ceter as 0 degrees. 
    private final int REFERENCE_POINT;

    public Turret(/*int mainCANId,*/ )
    {
        // NOTE ignore error, will deal with this later. 
        // NOTE May not be Talon control, if not talon control, then we will need to change the TalonSRX 
        super(TalonSRXFactory.buildFactoryTalonSRX(RobotMap.TALON_TURRET), 0, 180, 0, true, 0, 0, 0, 0, 0);         //TODO Complete this
        this.talonControl = super.rotator;
        REFERENCE_POINT =  super.getPositionTicks();
        
    }

    public double getAngleInDegrees()
    {
        //define degrees
        
        //there are 4096 ticks in a circle, so one degree is 11 17/45 ticks.
        return ((super.getPositionTicks() / TICKS_PER_REVOLUTION) * 360);
        // verify if this is counts per revolution as the input of get ticks in degrees. 
    }

    // /**
    //  * TODO: implement this method
    //  */
    // protected double getArbitraryFeedForwardAngleMultiplier()
    // {
    //     return 0.0;
    // }

    /**
     * angular speed, not linear speed
     * 
     * TODO Maybe change this, pretty obvious?!!
     * 
     * @return
     */

    // public double getSpeed() {
    //     //getTicksInDegrees
    //     //angle travelled / time

    //     return getEncoder().getPulseWidthVelocity();
    // }

    public int getEncoderPos() {
        return getEncoder().getPulseWidthPosition();
    }

//     /**
//      * motor health condition: 
//      *          indicates the current working capabilities, 
//      *                  raises flag when turret motor is not functioning properly. 
//      * 
//      * @return
//      */
//     public boolean motorHealthConditions() {
//         // talonControl 

//         //  This function is apparently depreciated
// //        double currentAmps = talonControl.getOutputCurrent();

//         final double currentAmps = talonControl.getSupplyCurrent();   // OR
// //        double currentAmps = talonControl.getStatorCurrent();

//         final double outputVoltage = talonControl.getMotorOutputVoltage();
//         final double busV = talonControl.getBusVoltage();
        
//         /**
//          * double quadEncoderPos = talonControl.getSelectedSensorPosition();
//          * 
//          */

//         return false;
//     }

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
    public void rotateTurret(/*double degrees*/) {

        /**
         * taking soft limit is 20 degrees
         * 
         * 1820 = 20 degrees
         *  
         * if ( 
         *      setpoint >= 1820 and super.getPositionTicks() >= 1820
         * ) then return
         * 
         * if (
         *      setpoint <= -1820 and super.getPositionTicks() <= -1820
         * ) then return
         * 
         * 
         */


        /**
         * 
         */
        if (Math.abs(setpoint) >= TURRET_LIMIT && Math.abs(super.getPositionTicks()) >= TURRET_LIMIT) 
        {
            return;
        }

        

        talonControl.set((ControlMode.Position), setpoint);//(degrees)/(getTicksInDegrees(ticksPerRevolution)));
         
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
     * physical limit of turret
     * 
     * @param previousTicks
     *      comparing the previous ticks to current ticks lets us know how much distance has been travelled over time, 
     *                  helps in telling weather the encoder is working especially when it stops when the motor is working. 
     * @return flaggingFunctionality
     *      to return weather there is any problems with the subsystem. 
     */
    // public boolean encoderHealthCondition(final double previousTicks) {
    //     final double numberOfTicks = getEncoderPos();

    //     // calculate potential travel in ticks
    //     final double power = getSpeed();
    //     return power != 0 && numberOfTicks != previousTicks;
    
    // }

    public double getTicksPerDegree() { //int numberOfTicks) {
        return TICKS_PER_REVOLUTION / 360;//((numberOfTicks / TICKS_PER_REVOLUTION) * 360);
    }

    public double getTicks(double degrees) {
        return degrees * getTicksPerDegree();
    }

    public double getDegrees(double ticks) {
        return ticks / getTicksPerDegree();
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public void setMotionMagicSetpointAngle(final double angle)
    {
        setpoint = (int)(angle*(getTicksPerDegree()));
    }

    public SensorCollection getEncoder() //   TODO This is not CANEncoder, this has to be changed later. 
    {
        return this.talonControl.getSensorCollection();                    // TODO FIX THIS
    }

}