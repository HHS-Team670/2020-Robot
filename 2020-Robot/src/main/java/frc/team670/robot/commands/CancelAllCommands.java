/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.DriveBase;

public class CancelAllCommands extends InstantCommand {

  public CancelAllCommands(DriveBase driveBase) {
    addRequirements(driveBase);
  }

  public void initialize() {
    SmartDashboard.putString("current-command", "CancelAllCommands");
  }

}
