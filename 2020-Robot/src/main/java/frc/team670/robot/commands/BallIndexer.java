/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.utils.Logger;

public class BallIndexer extends WaitCommand {
  private final Shooter shooter;

  public BallIndexer(Shooter shooter, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(seconds);
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    SmartDashboard.putString("current-command", "BallIndexer");
    shooter.setSpeed();
    Logger.consoleLog();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("current-command", "BallIndexer");
    shooter.setSpeed();
    Logger.consoleLog();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}