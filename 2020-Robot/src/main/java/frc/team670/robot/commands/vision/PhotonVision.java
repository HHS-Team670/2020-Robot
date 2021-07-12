package frc.team670.robot.commands.vision;
import edu.wpi.first.wpilibj.*;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.MustangController;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
public class PhotonVision {

    PhotonCamera camera;
    MustangController mController;

    public PhotonVision(){
        camera = new PhotonCamera("photonvision");
        mController = new MustangController(0);
    }

    public double[] getVisionValues() {
        final double CAMERA_HEIGHT_METERS = 0.9144;
        final double TARGET_HEIGHT_METERS = 32.9184;

        double distance = 0.0, angle = 0.0;

        // Angle between horizontal and the camera.
        final double CAMERA_PITCH_RADIANS = 0;
        if (mController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();
            if (result.hasTargets()) {
                // First calculate range
                distance = PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            result.getBestTarget().getPitch() * Math.PI/180);
                angle = result.getBestTarget().getYaw();
            }
        }
        // System.out.println("Angle: " + angle + ", Distance: " + distance);
        double[] visionValues = new double[] {angle, distance};
        return visionValues;


    }
}