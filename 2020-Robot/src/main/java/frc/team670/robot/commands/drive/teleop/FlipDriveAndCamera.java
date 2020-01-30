package frc.team670.robot.commands.drive.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.cameras.FlipCamera;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.RobotContainer;

public class FlipDriveAndCamera extends InstantCommand implements MustangCommand{

    public FlipDriveAndCamera() {
        super();
    }

    public void initialize() {
      SmartDashboard.putString("current-command", "FlipDriveAndCamera");
      boolean isReversed = XboxRocketLeagueDrive.isDriveReversed();

      // Matches camera direction to the new drive direction
        if(isReversed) {
            if(!FlipCamera.getCameraDirection()) {
                FlipCamera.flipCameraDirection();
            }
        } else {
            if(FlipCamera.getCameraDirection()) {
                FlipCamera.flipCameraDirection();
            }
        }

      // if (!isReversed) {
      //   Robot.leds.setReverseData(true);
      // } else {
      //   Robot.leds.setForwardData(true);
      // }
      XboxRocketLeagueDrive.setDriveReversed(!isReversed);
      RobotContainer.oi.rumbleDriverController(0.7, 0.2);
      Logger.consoleLog("Flipped Drive: %s", (!isReversed));
    }

}