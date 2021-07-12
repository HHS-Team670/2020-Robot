package frc.team670.robot.commands.vision;
import edu.wpi.first.wpilibj2.*;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

import org.photonvision.*;

public class PhotonVision {

    PhotonCamera camera;
    
    public PhotonVision(){
        camera = new PhotonCamera("photonvision");
    }

    public double[] getVisionValues() {
        final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

        // Angle between horizontal and the camera.
        final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                // First calculate range
                double range = PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));
            }
        }

    }
}