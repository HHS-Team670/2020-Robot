/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import frc.team670.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.utils.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * An example command that uses an example subsystem.
 */
public class AutoRunIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Intake intake;
  private double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoRunIntake(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("current-command", "AutoRunIntake");
    Logger.consoleLog();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // uses ir sensor to detect ball to deploy then roll/spin motorz(autonomous deploy)
  @Override
  public void execute() {
    if (!intake.isDeployed() && intake.getSensor()) {
      intake.setDeploy(true);
      intake.setRolling(speed, true); 
 
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!intake.getSensor()) {
      return true;
    }
    else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  public void end() {
    intake.setRolling(0, true);
    Logger.consoleLog();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
