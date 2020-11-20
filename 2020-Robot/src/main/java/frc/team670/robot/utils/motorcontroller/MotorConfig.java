package frc.team670.robot.utils.motorcontroller;

import java.util.HashMap;
import java.util.Map;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Settings and properties for specific kinds of motors.
 * 
 * @author ctychen
 */
public class MotorConfig {

    /**
     * Different motors
     */
    public enum Motor_Type {

        NEO, NEO_550, REDLINE_775PRO, BAG;

    }

    /**
     * Max current for each motor
     */
    public static final Map<Motor_Type, Integer> MOTOR_MAX_CURRENT = new HashMap<Motor_Type, Integer>() {
        {
            put(Motor_Type.NEO, 40);
            put(Motor_Type.NEO_550, 25);
            put(Motor_Type.REDLINE_775PRO, 30);
            put(Motor_Type.BAG, 30);
        }
    };

    /**
     * Motor type (Brushed vs Brushless)
     */
    public static final Map<Motor_Type, MotorType> MOTOR_TYPE = new HashMap<Motor_Type, MotorType>() {
        {
            put(Motor_Type.NEO, MotorType.kBrushless);
            put(Motor_Type.NEO_550, MotorType.kBrushless);
            put(Motor_Type.REDLINE_775PRO, MotorType.kBrushed);
            put(Motor_Type.BAG, MotorType.kBrushed);
        }
    };
}