package frc.team670.robot.utils.motorcontroller;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Wrapper class for a SparkMAX for reducing CAN bus overhead by skipping duplicate set
 * commands.
 * @author ctychen, lakshbhambhani
 */
public final class SparkMAXLite extends CANSparkMax{

    protected double lastSet = Double.NaN;
    protected ControlType lastControlType = null;
    protected CANSparkMax leader = null;

    /**
     * Creates a SparkMAX on a given ID. Defaults to brushless motor.
     */
    public SparkMAXLite(int id){
        super(id, MotorType.kBrushless);
    }

    /**
     * Creates a SparkMAX on a given ID.
     * @param Motortype type Whether the motor connected to the controller is a brushed or brushless motor
     */
    public SparkMAXLite(int id, MotorType type){
        super(id, type);
    }

    public double getLastSet(){
        return this.lastSet;
    }
    
    public ControlType getLastControlType(){
        return this.lastControlType;
    }

    /**
     * Applicable if this SparkMAX is set as a follower.
     * @return the 'leader' controller that the SparkMAX follows
     */
    public CANSparkMax getLeader(){
        return this.leader;
    }

    /**
     * @param leader the SparkMAX for this controller to follow, if applicable.
     */
    public void setFollow(CANSparkMax leader){
        this.leader = leader;
        super.follow(leader);
    }

    /**
     * @param ControlType mode for this motor controller
     * @param double value output of the controller, for the appropriate mode
     */
    public void set(ControlType mode, double value){
        if (value != lastSet || mode != lastControlType){
            this.lastSet = value;
            this.lastControlType = mode;
            super.getPIDController().setReference(value, mode);
        }
    }

    /**
     * Used to check if a sparkMax Motor Controller is connected successfully and has no error
     * @param sparkMax The motor which has to be checked for an error
     * @return
     */
    public static boolean isErrorFree(SparkMAXLite sparkMax){
        return (sparkMax != null && sparkMax.getLastError() != CANError.kOk);
    }

}
