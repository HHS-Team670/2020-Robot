/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.colorWheel;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.ColorWheelSpinner;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Spins the color wheel a certain number of times.
 * 
 * @author Antonio Cuan, Katelyn Yap, ctychen
 */
public class SpinColorWheel extends CommandBase implements MustangCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorWheelSpinner spinner;

  private int referenceColorNumber;
  private boolean isColorDetected;
  private int colorDetectedCount = 0;

  public SpinColorWheel(ColorWheelSpinner spinner) {
    this.spinner = spinner;
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorDetectedCount = 0;
    isColorDetected = false;
    referenceColorNumber = spinner.detectColor();
    spinner.setSpeed(spinner.MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.setSpeed(0.0);
  }

  /**
   * @return true when motor spins the color wheel enough times
   */
  @Override
  public boolean isFinished() {
    int detectedColorNumber = spinner.detectColor();

    if (detectedColorNumber == referenceColorNumber) { // when it detects the reference color, which is whatever color
                                                       // it sees first
      isColorDetected = true;
    }

    if (isColorDetected && detectedColorNumber != referenceColorNumber) { // has detected ref. color; the wheel is at
                                                                          // the next color
      colorDetectedCount++;
      isColorDetected = false;
    }

    if (colorDetectedCount == 7) { // 7 means 3+(1/8) rotations and 1 is added to colorDetectedCount once 1/8 of a
                                   // rotation is completed
      return true;
    }

    return false;
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    Map<MustangSubsystemBase, HealthState> healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(spinner, HealthState.GREEN);
    return healthReqs;
  }
}
