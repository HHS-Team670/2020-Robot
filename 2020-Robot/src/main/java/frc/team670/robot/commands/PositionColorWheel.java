/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import frc.team670.robot.subsystems.ColorWheelSpinner;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.utils.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A PositionColorWheel command that uses a ColorWheelSpinner subsystem.
 */
public class PositionColorWheel extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorWheelSpinner m_spinner;

  private double motorSpeed = 1.0;
  private int resultColorNumber;
  private final static int OFFSET_SIZE = 2; // note that if they offset is three or one, the program will only work if
                                            // the color sensor is at a certain position

  /**
   * Creates a new PositionColorWheel command.
   *
   * @param ColorWheelSpinner The subsystem used by this command.
   */
  public PositionColorWheel(ColorWheelSpinner spinner) {
    m_spinner = spinner;
    // SmartDashboard.putNumber("Target Color Number", -1);
    // SmartDashboard.putBoolean("isSpinning", false);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int targetColorNumber = -1;
    // int targetColorNumber = (int) SmartDashboard.getNumber("Target Color Number", -1);

    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          targetColorNumber = 0;
          break;
        case 'Y':
          targetColorNumber = 1;
          break;
        case 'R':
          targetColorNumber = 2;
          break;
        case 'G':
          targetColorNumber = 3;
          break;
        default:
        Logger.consoleLog("This is corrupt data");
          break;
        }
    } else {
      Logger.consoleLog("No data received.");
    }

    resultColorNumber = (((targetColorNumber) + OFFSET_SIZE) % 4); /** 
                                                                   * calculates offset color number since the robot
                                                                   * color sensor is in a different place than the frc
                                                                   * sensor on the color wheel;
                                                                   */
    SmartDashboard.putNumber("result color number", resultColorNumber);
    m_spinner.setSpeed(motorSpeed);
    // SmartDashboard.putBoolean("isSpinning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.setSpeed(0.0);
    // SmartDashboard.putBoolean("isSpinning", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int detectedColorNumber = m_spinner.detectColor();

    if (detectedColorNumber == resultColorNumber) {
      return true;
    }

    return false;
  }
}
