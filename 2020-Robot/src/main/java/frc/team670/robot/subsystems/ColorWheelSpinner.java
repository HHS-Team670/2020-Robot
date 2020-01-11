package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ColorWheelSpinner extends SubsystemBase {

    private String referenceColor = "";
    private boolean sawColor;
    public int sawColorCount = 0;
    private double motorSpeed = 0.8;
    private boolean rotationControlTriggered = false;
    private VictorSPX victor;

    public ColorWheelSpinner() {
        victor = new VictorSPX(1); //TODO: Constructor takes in CAN ID
    }

    public void whenButtonPressed() {
        rotationControlTriggered = true;
        referenceColor = RobotContainer.colorMatch.colorSeen;
        victor.set(ControlMode.PercentOutput, motorSpeed);
    }


    public void periodic() {
        if (rotationControlTriggered) {
            if (RobotContainer.colorMatch.colorSeen.equals(referenceColor)) {
                sawColor = true;
            }

            if (sawColor && RobotContainer.colorMatch.colorSeen != referenceColor) {
                sawColorCount ++;
                sawColor = false;
            }

            if (sawColorCount == 9) {
                victor.stopMotor();
            }
        }   
    }
}