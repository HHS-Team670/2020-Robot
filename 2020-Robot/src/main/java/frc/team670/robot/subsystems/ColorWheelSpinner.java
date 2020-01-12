package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.constants.RobotConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ColorWheelSpinner extends SubsystemBase {

    private VictorSPX victor;
    
    public void setSpeed(double motorSpeed) {
        victor.set(ControlMode.PercentOutput, motorSpeed);
    }

    public int detectColor() {
        return RobotContainer.colorMatch.detectColor();
    }

    public ColorWheelSpinner() {
        victor = new VictorSPX(RobotConstants.COLOR_WHEEL_MOTOR_CAN_ID); // Constructor takes in CAN ID
    }
}