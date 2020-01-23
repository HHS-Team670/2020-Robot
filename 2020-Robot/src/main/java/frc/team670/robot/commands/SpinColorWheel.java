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
 * An SpinColorWheel command that uses an ColorWheelSpinner subsystem.
 */
public class SpinColorWheel extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ColorWheelSpinner m_spinner;

  private double motorSpeed = 1.0;
  private int referenceColorNumber;
  private boolean isColorDetected;
  private int colorDetectedCount = 0;

  /**
   * Creates a new SpinColorWheel command.
   *
   * @param ColorWheelSpinner The subsystem used by this command.
   */
  public SpinColorWheel(ColorWheelSpinner spinner) {
    m_spinner = spinner;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorDetectedCount = 0;
    isColorDetected = false;
    SmartDashboard.putNumber("color detected count", 0);   
    referenceColorNumber = m_spinner.detectColor();
    m_spinner.setSpeed(motorSpeed);
    SmartDashboard.putBoolean("isSpinning", true);    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.setSpeed(0.0);
    SmartDashboard.putBoolean("isSpinning", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int detectedColorNumber = m_spinner.detectColor();

    if (detectedColorNumber == referenceColorNumber) { // when it detects the reference color(which is whatever color it sees first)
      isColorDetected = true;
    }

    if (isColorDetected && detectedColorNumber != referenceColorNumber) { // has detected ref. color; the wheel is at the next color 
      colorDetectedCount ++;
      isColorDetected = false;   
      SmartDashboard.putNumber("color detected count", colorDetectedCount);   
    }

    if (colorDetectedCount == 7) { // 7 means 3+(1/8) rotations and 1 is added to colorDetectedCount once 1/8 of a rotation is completed  
      return true;      
    }    

    return false;
  }
}
