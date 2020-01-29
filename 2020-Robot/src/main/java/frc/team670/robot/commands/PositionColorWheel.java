/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import frc.team670.robot.dataCollection.sensors.ColorMatcher;
import frc.team670.robot.subsystems.ColorWheelSpinner;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.utils.Logger;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Spins the color wheel to a specified color from the field.
 * 
 * @author Antonio Cuan, Katelyn Yap 
 */
public class PositionColorWheel extends MustangCommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorWheelSpinner m_spinner;

  private int resultColorNumber;
  private final int COLOR_COUNT = 4; 
  private final int OFFSET_SIZE = 2; // note that if they offset is three or one, the program will only work if the color sensor is at a certain position
  // TODO: tune the offset number
  /**
   * Creates a new PositionColorWheel command.
   *
   * @param ColorWheelSpinner The subsystem used by this command.
   */
  public PositionColorWheel(ColorWheelSpinner spinner) {
    m_spinner = spinner;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int targetColorNumber = ColorMatcher.UNKNOWN_COLOR_NUMBER;

    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        targetColorNumber = ColorMatcher.colors.BLUE.getColorNumber();
        Logger.consoleLog("Game data: %s", gameData);
        break;
      case 'Y':
        targetColorNumber = ColorMatcher.colors.YELLOW.getColorNumber();
        Logger.consoleLog("Game data: %s", gameData);
        break;
      case 'R':
        targetColorNumber = ColorMatcher.colors.RED.getColorNumber();
        Logger.consoleLog("Game data: %s", gameData);
        break;
      case 'G':
        targetColorNumber = ColorMatcher.colors.GREEN.getColorNumber();
        Logger.consoleLog("Game data: %s", gameData);
        break;
      default:
        Logger.consoleLog("This is corrupt data.");
        Logger.consoleLog(gameData);
        MustangScheduler.getInstance().cancel(this);
        break;
      }
    } else {
      Logger.consoleLog("No data received.");
    }

    resultColorNumber = (((targetColorNumber) + OFFSET_SIZE) % COLOR_COUNT); 
    /**
    * calculates offset color number since the robot color sensor is in a different
    * place than the frc sensor on the color wheel;
    */
    Logger.consoleLog("Color to detect: %s", resultColorNumber);
    m_spinner.setSpeed(m_spinner.MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.setSpeed(0.0);
  }

  /**
   * @return true when detects desired color
   */
  @Override
  public boolean isFinished() {
    int detectedColorNumber = m_spinner.detectColor();

    if (detectedColorNumber == resultColorNumber) {
      return true;
    }

    return false;
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return null;
  }
}
