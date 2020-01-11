package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ColorWheelPositioner extends SubsystemBase {

    private boolean positionControlTriggered = false;
    private double motorSpeed;
    public String priorityColor = "green";
    private VictorSPX victor;

    public ColorWheelPositioner() {
        victor = new VictorSPX(1); //TODO: Constructor takes in CAN ID
    }

    public void whenButtonPressed() {
        positionControlTriggered = true;
        victor.set(ControlMode.PercentOutput, motorSpeed);
    }

    public void periodic() {
        if (positionControlTriggered) {
            if (RobotContainer.colorMatch.colorSeen.equals(priorityColor)) {
                victor.stopMotor();
            }
        }
    }
}