package frc.team670.robot.subsystems;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.*;

//TODO: Change all of this to using a SparkMax instead of TalonSRX

public class Turret extends SparkMaxRotatingSubsystem {
    private final CANSparkMax sparkControl;
    private final int TICKS_PER_REVOLUTION = 4096;; //ticks per revolution
    // unused private final int TURRET_LIMIT = (int)((TICKS_PER_REVOLUTION * 160.0)/360.0);                  // assuming the turret's soft limit will be 1820 ticks = 160 degrees both ways from center as 0 degrees.           
    private double referencePoint;
    private final double SOFT_LIMIT_IN_DEGREES = 160.0;

    private final double HARD_LIMIT_IN_DEGREES = 180.0;

    public Turret(/*int mainCANId,*/ ) {
        // NOTE ignore error, will deal with this later. 
        // NOTE May not be Talon control, if not talon control, then we will need to change the TalonSRX 
        //super(TalonSRXFactory.buildFactoryTalonSRX(RobotMap.TALON_TURRET), 0, 180, 0, true, 0, 0, 0, 0, 0);         //TODO Complete this
        super(SparkMAXFactory.buildFactorySparkMAX(RobotMap.SPARK_TURRET), 0);
        this.sparkControl = super.rotator;
        this.sparkControl.setSoftLimit(SoftLimitDirection.kForward, (float)getTicks(SOFT_LIMIT_IN_DEGREES));
        this.sparkControl.setSoftLimit(SoftLimitDirection.kReverse, (float)getTicks(SOFT_LIMIT_IN_DEGREES));
        this.sparkControl.enableSoftLimit(SoftLimitDirection.kForward, true);
        this.sparkControl.enableSoftLimit(SoftLimitDirection.kReverse, true);
               //referencePoint = super.encoder.getPosition();
        
    }

    /**
     * @return the referencePoint
     */
    public void setZeroPoint(double point) {
        referencePoint = point;
    }

    /*

    */


    /**
     * @return the referencePoint
     */
    public double getZeroPoint() {
        return referencePoint;
    }

    /**
     * 
     * @return the position of the turret in degrees
     */
    public double getAngleInDegrees() {
        //define degrees
        
        //there are 4096 ticks in a circle, so one degree is 11 17/45 ticks.
        return ((getEncoderPos() / TICKS_PER_REVOLUTION) * 360);
        // verify if this is counts per revolution as the input of get ticks in degrees. 
    }



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

    public double getEncoderPos() {
        
        return this.encoder.getPosition() - referencePoint;// - referencePoint;
    }

    public CANPIDController getPIDController() {
        return this.rotator.getPIDController();
    }

    /**
     * 
     * Takes in degrees and rotates the turret to that position
     * 
     * 
     * @param degrees the position in degrees the turret should move
     *      
     */
    public void setTurretSpeed(double speed) {

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
/*
         if(Math.abs(degrees) > SOFT_LIMIT_IN_DEGREES) {
             return;
         }

        setpoint = (int)(degrees*getTicksPerDegree());

        // if (Math.abs(setpoint) >= HARD_LIMIT && Math.abs(super.getPositionTicks()) >= HARD_LIMIT) {
        //     return;
        // }
*/
        
        this.rotator.set(speed);
        //set((ControlMode.Position), setpoint);//(degrees)/(getTicksInDegrees(ticksPerRevolution)));
         
        // TODO research this and make changes, this section is not complete. 
     }


    /**
     * Rotates the turret amount amount.
     * @param amount the amount (in degrees) the turret should rotate
     */
    /*public void rotateTurretAdditionalAmount(double amount) {
        if(Math.abs(getAngleInDegrees() + amount) > SOFT_LIMIT_IN_DEGREES)
        {
            return;
        }
        double amountInTicks = amount*getTicksPerDegree();
        sparkControl.set((ControlMode.Position), super.encoder.getPosition() + amountInTicks);
    }*/

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
    public void setMotionMagicSetpointAngle(final double angle) {
        setpoint = (int)(angle*(getTicksPerDegree()));
    }

    public CANEncoder getEncoder() { //   TODO This is not CANEncoder, this has to be changed later.
        return this.encoder;                    // TODO FIX THIS
    }

    @Override
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void moveByPercentOutput(double output) {
        // TODO Auto-generated method stub

    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }



    //  Unused methods for potential later use. 


    // /**
    //  * TODO: implement this method
    //  */
    // protected double getArbitraryFeedForwardAngleMultiplier()
    // {
    //     return 0.0;
    // }

    //     /**
//      * motor health condition: 
//      *          indicates the current working capabilities, 
//      *                  raises flag when turret motor is not functioning properly. 
//      * 
//      * @return
//      */
//     public boolean motorHealthConditions() {
//         // sparkControl 

//         //  This function is apparently depreciated
// //        double currentAmps = sparkControl.getOutputCurrent();

//         final double currentAmps = sparkControl.getSupplyCurrent();   // OR
// //        double currentAmps = sparkControl.getStatorCurrent();

//         final double outputVoltage = sparkControl.getMotorOutputVoltage();
//         final double busV = sparkControl.getBusVoltage();
        
//         /**
//          * double quadEncoderPos = sparkControl.getSelectedSensorPosition();
//          * 
//          */

//         return false;
//     }

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


}