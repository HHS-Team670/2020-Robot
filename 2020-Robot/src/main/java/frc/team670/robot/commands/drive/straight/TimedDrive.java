/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.straight;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;



public class TimedDrive extends WaitCommand {

  private DriveBase driveBase;

  public TimedDrive(DriveBase driveBase, int seconds) {
    super(seconds);
    this.driveBase = driveBase;
    addRequirements(this.driveBase);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    driveBase.tankDrive(0.5, 0.5);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean isInteruppted) {
    driveBase.stop();
    Logger.consoleLog("Drivebase stopped");
  }

}
