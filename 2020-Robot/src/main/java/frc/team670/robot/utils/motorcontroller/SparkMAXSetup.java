package frc.team670.robot.utils.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Wrapper class for a SparkMAX for reducing CAN bus overhead by skipping duplicate set
 * commands.
 */
public final class SparkMAXSetup extends CANSparkMax{

    protected double lastSet = Double.NaN;
    protected ControlType lastControlType = null;
    protected CANSparkMax master = null;

    public SparkMAXSetup(int id){
        super(id, MotorType.kBrushless);
    }

    public double getLastSet(){
        return this.lastSet;
    }
    
    public ControlType getLastControlType(){
        return this.lastControlType;
    }

    /**
     * Applicable if this SparkMAX is set as a follower.
     * @return the 'master' controller that the SparkMAX follows
     */
    public CANSparkMax getMaster(){
        return this.master;
    }

    /**
     * @param master the SparkMAX for this controller to follow, if applicable.
     */
    public void setFollow(CANSparkMax master){
        this.master = master;
        super.follow(master);
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

}
