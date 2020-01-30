/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.subsystems.ColorWheelSpinner;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Spins the color wheel at a slower speed when the driver presses a button
 * (manual spin), use if something breaks.
 * 
 * @author Antonio Cuan
 */
public class ManualSpinColorWheel extends CommandBase implements MustangCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ColorWheelSpinner m_spinner;

  private double MOTOR_SPEED = 0.5; // TODO: tune speed

  /**
   * Creates a new ManualSpinColorWheel.
   *
   * @param ColorWheelSpinner The subsystem used by this command.
   */
  public ManualSpinColorWheel(ColorWheelSpinner spinner) {
    m_spinner = spinner;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.oi.isManualSpinColorWheelButtonPressed()) {
      m_spinner.setSpeed(MOTOR_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!RobotContainer.oi.isManualSpinColorWheelButtonPressed()) {
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
