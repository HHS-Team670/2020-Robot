/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.straight;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

public class TimedDrive extends WaitCommand implements MustangCommand {

  private DriveBase driveBase;

  public TimedDrive(int seconds, DriveBase driveBase) {
    super(seconds);
    addRequirements(driveBase);
    this.driveBase = driveBase;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Logger.consoleLog("TimedDrive executed");
    driveBase.tankDrive(0.3, 0.3);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean isInteruppted) {
    driveBase.stop();
    Logger.consoleLog("Robot stopped");
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
    healthRequirements.put(driveBase, HealthState.YELLOW);
    return healthRequirements;
  }
}