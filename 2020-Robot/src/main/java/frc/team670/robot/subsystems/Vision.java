package frc.team670.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author ctychen, lakshbhambhani
 */
public class Vision extends MustangSubsystemBase{

    private Solenoid cameraLEDs;

    PhotonCamera camera = new PhotonCamera("photonvision");

    // These are for sending vision health to dashboard
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();


    public double getAngleToTarget(){
        return instance.getTable("photonvision").getSubTable("Microsoft_LifeCam_HD-3000").getEntry("targetYaw").getDouble(-1);
    }

    public double getDistanceToTargetM(){
        return getDistanceToTargetInches() * 2.54 / 100;
    }

    public boolean hasTarget(){
        return camera.getLatestResult().hasTargets();
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    public double getDistanceToTargetInches() {
        // return getDistanceToTargetCm() / 2.54;
        var result = camera.getLatestResult();

        double range =
        PhotonUtils.calculateDistanceToTargetMeters(
                37.5,
                58,
                RobotConstants.TILT_ANGLE,
                Units.degreesToRadians(result.getBestTarget().getPitch()));

            return range;
    }

    /**
     * 
     * @return distance, in cm, from the camera to the target
     */
    public double getDistanceToTargetCm() {
        return getDistanceToTargetM() * 100;
    }

    public void turnOnLEDs() {
        cameraLEDs.set(true);
    }

    public void turnOffLEDs() {
        cameraLEDs.set(false);
    }

    public void testLEDS() {
        cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
    }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        SmartDashboard.putNumber("Distance", getDistanceToTargetInches());
        SmartDashboard.putNumber("Angle", getAngleToTarget());
    }

}