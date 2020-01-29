package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.ColorMatcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ColorWheelSpinner extends SubsystemBase {

    private static ColorMatcher colorMatch;
    private CANSparkMax rotator;

    public final double MOTOR_SPEED = 1.0; // TODO: tune for actual motor

    public ColorWheelSpinner() {
        colorMatch = new ColorMatcher();
        colorMatch.init();
        rotator = new CANSparkMax(RobotMap.COLOR_WHEEL_MOTOR_CAN_ID, MotorType.kBrushless); // Constructor takes in CAN ID
    }
    
    public void setSpeed(double motorSpeed) {
        rotator.set(motorSpeed);
    }

    public int detectColor() {
        return colorMatch.detectColor();
    }    
}