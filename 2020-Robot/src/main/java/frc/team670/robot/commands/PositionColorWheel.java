/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import frc.team670.robot.subsystems.ColorWheelSpinner;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class PositionColorWheel extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ColorWheelSpinner m_spinner;

  private double motorSpeed = 1.0;
  private double targetColorNumber;
  private int offsetSize = 3;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionColorWheel(ColorWheelSpinner spinner) {
    m_spinner = spinner;
    SmartDashboard.putNumber("Target Color Number", 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    m_spinner.setSpeed(motorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_spinner.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    targetColorNumber = SmartDashboard.getNumber("Target Color Number", 0);
    
    int detectedColorNumber = m_spinner.detectColor();

    int resultColorNumber = detectedColorNumber + offsetSize;
    if (resultColorNumber > 4) {
      int difference = (resultColorNumber - detectedColorNumber) - 1;
      resultColorNumber = 1 + difference;
    }

    if (detectedColorNumber == targetColorNumber) {
      return true;
    }

    return false;
  }
}