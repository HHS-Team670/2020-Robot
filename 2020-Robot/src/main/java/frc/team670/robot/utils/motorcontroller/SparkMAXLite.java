package frc.team670.robot.utils.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Wrapper class for a SparkMAX for reducing CAN bus overhead by skipping
 * duplicate set commands.
 * 
 * @author ctychen, lakshbhambhani
 */
public final class SparkMAXLite extends CANSparkMax {

    protected double lastSet = Double.NaN;
    protected ControlType lastControlType = null;
    protected CANSparkMax leader = null;
    protected MotorConfig.Motor_Type motorType = null;

    /**
     * Creates a SparkMAX on a given ID, which is controlling a specified kind of motor.
     * 
     * @param MotorConfig.Motor_Type type Which specific motor this controller will
     *                               be using. For example, NEO or BAG
     */
    public SparkMAXLite(int id, MotorConfig.Motor_Type type) {
        super(id, MotorConfig.MOTOR_TYPE.get(type));
        this.motorType = type;
    }

    public double getLastSet() {
        return this.lastSet;
    }

    public ControlType getLastControlType() {
        return this.lastControlType;
    }

    public MotorConfig.Motor_Type getMotor(){
        return this.motorType;
    }

    /**
     * Applicable if this SparkMAX is set as a follower.
     * 
     * @return the 'leader' controller that the SparkMAX follows
     */
    public CANSparkMax getLeader() {
        return this.leader;
    }

    /**
     * @param leader the SparkMAX for this controller to follow, if applicable.
     */
    public void setFollow(CANSparkMax leader) {
        this.leader = leader;
        super.follow(leader);
    }

    /**
     * @param ControlType mode for this motor controller
     * @param double      value output of the controller, for the appropriate mode
     */
    public void set(ControlType mode, double value) {
        if (value != lastSet || mode != lastControlType) {
            this.lastSet = value;
            this.lastControlType = mode;
            super.getPIDController().setReference(value, mode);
        }
    }

}
