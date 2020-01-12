package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.ColorMatcher;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ColorWheelSpinner extends SubsystemBase {

    private static ColorMatcher colorMatch;
    private VictorSPX victor;

    public ColorWheelSpinner() {
        colorMatch = new ColorMatcher();
        victor = new VictorSPX(RobotMap.COLOR_WHEEL_MOTOR_CAN_ID); // Constructor takes in CAN ID
    }
    
    public void setSpeed(double motorSpeed) {
        victor.set(ControlMode.PercentOutput, motorSpeed);
    }

    public int detectColor() {
        return colorMatch.detectColor();
    }    
}