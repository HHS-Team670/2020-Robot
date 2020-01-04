/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.Robot;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.utils.functions.JoystickUtils;

 

/**
 * Drives the Robot using Xbox controls like the game Rocket League. Triggers control speed, stick is for steering.
 * @author lakshbhambhani
 */
public class XboxRocketLeagueDrive extends InstantCommand {

   private final boolean SMOOTH_ROCKET_LEAGUE_STEER, SMOOTH_ROCKET_LEAGUE_TRIGGER;
   private static boolean isReversed;

  public XboxRocketLeagueDrive() {
    super();
    SMOOTH_ROCKET_LEAGUE_STEER = true;
    SMOOTH_ROCKET_LEAGUE_TRIGGER = true;
    isReversed = false;
    addRequirements(RobotContainer.driveBase);
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    // Sets the speed to the reading given by the trigger axes on the controller. Left is positive, but we multiply
    // by -1 to reverse that because we want right trigger to correspond to forward.
    double speed = -1 * (RobotContainer.oi.getDriverController().getLeftTriggerAxis() - RobotContainer.oi.getDriverController().getRightTriggerAxis()); 
    double steer = RobotContainer.oi.getDriverController().getLeftStickX(); 
   

    // Decides whether or not to smooth the Steering and Trigger. Smoothing helps
    // reduce jerkiness when driving.
    // tankDrive actually does this for us automatically, so npo need to do it
    // ourselves
    steer = JoystickUtils.smoothInput(steer);
    speed = JoystickUtils.smoothInput(speed);

    if (isReversed) {
      steer *= -1;
      speed *= -1;
    }

    if(RobotContainer.oi.isQuickTurnPressed()){

      if(speed < -0.0001) {
        if(!isReversed) {
          RobotContainer.driveBase.curvatureDrive(speed, -1 * steer, RobotContainer.oi.isQuickTurnPressed());
        }
        else {
          RobotContainer.driveBase.curvatureDrive(speed, -1 * steer, RobotContainer.oi.isQuickTurnPressed());
        }
      }
      else if (speed > 0.0001){
        if(!isReversed) {
          RobotContainer.driveBase.curvatureDrive(speed, steer, RobotContainer.oi.isQuickTurnPressed());
        }
        else {
          RobotContainer.driveBase.curvatureDrive(speed, steer, RobotContainer.oi.isQuickTurnPressed());
        }
      } else {
        if(!isReversed) {
          RobotContainer.driveBase.curvatureDrive(speed, steer, RobotContainer.oi.isQuickTurnPressed());
        }
        else {
          RobotContainer.driveBase.curvatureDrive(speed, -1 * steer, RobotContainer.oi.isQuickTurnPressed());
        }
      }
    }
    else {
      if (speed < -0.0001){
        RobotContainer.driveBase.curvatureDrive(speed, -1 * steer, RobotContainer.oi.isQuickTurnPressed());
      } else {
        RobotContainer.driveBase.curvatureDrive(speed, steer, RobotContainer.oi.isQuickTurnPressed());
      }
    }
  }

  public static boolean isDriveReversed() {
    return isReversed;
  }

  public static void setDriveReversed(boolean reversed) {
    XboxRocketLeagueDrive.isReversed = reversed;
    SmartDashboard.putBoolean("drive-reversed", reversed);
  }

}