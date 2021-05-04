/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.cameras;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.RobotContainer;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

/**
 * Flips between single camera and double camera on the dashboard
 */
public class FlipDriverCameraMode extends InstantCommand implements MustangCommand {
  /**
   * Add your docs here.
   */
  private static boolean doubleCamera = true;

  public FlipDriverCameraMode() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  public void initialize() {
    doubleCamera = !doubleCamera;
    SmartDashboard.putString("driver-camera-mode", doubleCamera ? "double" : "single");
    RobotContainer.rumbleDriverController();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return null;
  }

}
