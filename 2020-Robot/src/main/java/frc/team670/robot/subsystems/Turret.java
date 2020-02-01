package frc.team670.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.*;


public class Turret extends SparkMaxRotatingSubsystem {
    private final CANSparkMax sparkControl;
    public final int TICKS_PER_REVOLUTION = 4096;      
    private CANPIDController pidController;
    private final double SOFT_LIMIT_IN_DEGREES = 160.0;
    private final double P = 0;
    private final double I = 0;
    private final double D = 0;

    public Turret() {
       
        
        super(SparkMAXFactory.buildFactorySparkMAX(RobotMap.SPARK_TURRET), 0);
        this.sparkControl = super.rotator;
        this.sparkControl.setSoftLimit(SoftLimitDirection.kForward, (float)getTicks(SOFT_LIMIT_IN_DEGREES));
        this.sparkControl.setSoftLimit(SoftLimitDirection.kReverse, (float)getTicks(SOFT_LIMIT_IN_DEGREES));
        this.sparkControl.enableSoftLimit(SoftLimitDirection.kForward, true);
        this.sparkControl.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        pidController = new CANPIDController(sparkControl);
        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);
        
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



    public void rotateToAngle(double setpoint) {
        pidController.setReference(getTicks(setpoint), ControlType.kPosition);
    }
  

    
    /**
     * 
     * @return encoder position in ticks
     */
    public double getEncoderPos() {
        
        return this.encoder.getPosition();// - referencePoint;
    }

    public CANPIDController getPIDController() {
        return this.rotator.getPIDController();
    }

    /**
     * 
     * takes in double from -1 to 1 to set speed of turret motor
     * 
     * @param speed speed to set turret to    
     */
    
    public void setTurretSpeed(double speed) {
        
        this.rotator.set(speed);
  
     }


    
     /**
      * 
      * @return double ratio of ticks to degrees. Can be multiplied by degrees to get ticks
      */
    public double getTicksPerDegree() { 
        return TICKS_PER_REVOLUTION / 360;
    }

    /**
     * Converts degrees to ticks
     * @param degrees value to convert
     * @return
     */
    public double getTicks(double degrees) {
        return degrees * getTicksPerDegree();
    }

    /**
     * Converts ticks to degrees
     * @param ticks value to convert
     */
    public double getDegrees(double ticks) {
        return ticks / getTicksPerDegree();
    }

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    /*public void setMotionMagicSetpointAngle(final double angle) {
        setpoint = (int)(angle*(getTicksPerDegree()));
    }*/

    /**
     * @return the encoder of the motor
     */
    public CANEncoder getEncoder() { 
        return this.encoder;                   
    }

    /**
     * @return ???
     */
    @Override
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

    /**
     * Abstract method we have no clue what to do with
     */
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