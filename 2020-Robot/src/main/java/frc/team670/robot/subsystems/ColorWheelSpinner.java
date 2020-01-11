package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
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

    public String detectColor() {
        return RobotContainer.colorMatch.detectColor();
    }

    public void moveToColor(String color) {
        setSpeed(0.5);
        while (!detectColor().equals(color)) {}
        setSpeed(0);
    }

    public void rotate(int rotations) {
        String c = detectColor();
        int i = 1;
        setSpeed(0.5);
        while (i < rotations*2)  {
            if (detectColor().equals(c))
                i++;
        }
        setSpeed(0);
    }

    // public void rotateSection(int sections) { //comment out for testing
    //     String c = detectColor();
    //     int i = 0;
    //     while(detectColor().equals(c)) {
    //     }
    // }

    public ColorWheelSpinner() {
        victor = new VictorSPX(RobotConstants.COLOR_WHEEL_MOTOR_CAN_ID); //TODO: Constructor takes in CAN ID
    }
}