/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import frc.team670.robot.subsystems.ColorWheelSpinner;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Spins the color wheel a certain number of times.
 * 
 * @author Antonio_Cuan, Katelyn_Yap, ctychen
 */
public class SpinColorWheel extends MustangCommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorWheelSpinner m_spinner;

  private int referenceColorNumber;
  private boolean isColorDetected;
  private int colorDetectedCount = 0;

  public SpinColorWheel(ColorWheelSpinner spinner) {
    m_spinner = spinner;
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorDetectedCount = 0;
    isColorDetected = false;
    referenceColorNumber = m_spinner.detectColor();
    m_spinner.setSpeed(m_spinner.MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.setSpeed(0.0);
  }

  /**
   * @return true when ..
   */
  @Override
  public boolean isFinished() {
    int detectedColorNumber = m_spinner.detectColor();

    if (detectedColorNumber == referenceColorNumber) { // when it detects the reference color, which is whatever color it sees first
      isColorDetected = true;
    }

    if (isColorDetected && detectedColorNumber != referenceColorNumber) { // has detected ref. color; the wheel is at the next color
      colorDetectedCount++;
      isColorDetected = false;
    }

    if (colorDetectedCount == 7) { // 7 means 3+(1/8) rotations and 1 is added to colorDetectedCount once 1/8 of a rotation is completed
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
