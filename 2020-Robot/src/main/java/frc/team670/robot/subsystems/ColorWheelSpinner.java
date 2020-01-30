package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.ColorMatcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ColorWheelSpinner extends MustangSubsystemBase {

    private static ColorMatcher colorMatch;
    private CANSparkMax rotator;

    public final double MOTOR_SPEED = 0.1; // TODO: tune for actual motor

    public ColorWheelSpinner() {
        colorMatch = new ColorMatcher();
        rotator = new CANSparkMax(RobotMap.COLOR_WHEEL_MOTOR_ID, MotorType.kBrushless); // Constructor takes in CAN ID
    }

    public void setSpeed(double motorSpeed) {
        rotator.set(motorSpeed);
    }

    public int detectColor() {
        return colorMatch.detectColor();
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }
}