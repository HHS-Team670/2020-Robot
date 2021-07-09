package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.robot.commands.vision.PhotonVision;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author lakshbhambhani, katia, pallavi
 */
public class VisionSubsystemBase extends MustangSubsystemBase {

    private Solenoid cameraLEDs;
    private PhotonVision photonvision;

    private final String VISION_TRIGGER_KEY = "vision-enabled";
    public final static String VISION_VALUES_KEY = "vision-values";

    private final String VISION_MIN_HSV_BASE_KEY = "vision-min-";
    private final String VISION_MAX_HSV_BASE_KEY = "vision-max-";
    private final String[] VISION_HSV_SUB_KEY = new String[] {"h", "s", "v"};
    
    private final String VISION_SHAPE_BASE_KEY = "vision-shape-"; // labeled as point#-x
    private final String[] VISION_SHAPE_SUB_KEY = new String[] {"x", "y", "z"};

    private final String VISION_HEALTH_KEY = "vision-health";

    private double angle;
    private double distance;

    /**
     * Used to create a mustangCoprocessor object based on the key of the table that returns the values from vision processing
     */
    public VisionSubsystemBase(double[] minHSV, double[] maxHSV, double[] ds, int PCMModulePort, int visionLEDPCMPort) {
        photonvision = new PhotonVision();
        cameraLEDs = new Solenoid(PCMModulePort, visionLEDPCMPort);
        SmartDashboard.putBoolean("LEDs on", false);
        
        for(int i = 0; i < VISION_HSV_SUB_KEY.length; i++) {
            SmartDashboard.putNumber(VISION_MIN_HSV_BASE_KEY + VISION_HSV_SUB_KEY[i], minHSV[i]);
            SmartDashboard.putNumber(VISION_MAX_HSV_BASE_KEY + VISION_HSV_SUB_KEY[i], maxHSV[i]);
        }

        for(int i = 0; i < ds.length; i++) {
            SmartDashboard.putNumber(VISION_SHAPE_BASE_KEY + i + VISION_SHAPE_SUB_KEY[0], -1);
            SmartDashboard.putNumber(VISION_SHAPE_BASE_KEY + i + VISION_SHAPE_SUB_KEY[1], -1);
            SmartDashboard.putNumber(VISION_SHAPE_BASE_KEY + i + VISION_SHAPE_SUB_KEY[2],  -1);
        }

    }

    /**
     * Used to trigger the vision system to run and get new values
     */
    public void triggerVision() {
        SmartDashboard.putBoolean(VISION_TRIGGER_KEY, true);
    }

    /**
     * Used to trigger the vision system to run and get new values
     */
    public void getLatestVisionData() {
        clearLastValues();
        // Double[] values = SmartDashboard.getNumberArray(VISION_VALUES_KEY, new Double[] {-1.0,-1.0});
        double[] values = photonvision.getVisionValues();
        angle = values[0];
        distance = values[1];
    }


    /**
     * Used to clear last values present on the table
     */
    public void clearLastValues(){
        SmartDashboard.putNumberArray(VISION_VALUES_KEY, new Double[] {-1.0,-1.0});
    }

    /**
     * 
     * @return the horizontal angle between the camera-forward to the robot-target line
     */
    public double getAngleToTarget() {
        getLatestVisionData();
        return angle;
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    private double getDistanceToTargetInches() {
        getLatestVisionData();
        return distance;
    }

    /**
     * 
     * @return distance, in cm, from the camera to the target
     */
    public double getDistanceToTargetMeters() {
        return getDistanceToTargetInches() * 25.4;
    }

    /**
     * Used to turn on the bright green leds for vision
     */
    public void turnOnLEDs() {
        cameraLEDs.set(true);
    }

    /**
     * Used to turn off the bright green leds for vision
     */
    public void turnOffLEDs() {
        cameraLEDs.set(false);
    }

    /**
     * Used to test the leds for vision
     */
    public void testLEDS() {
        cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
    }

    @Override
    public HealthState checkHealth() {
        HealthState state;

        String visionHealth = SmartDashboard.getString(VISION_HEALTH_KEY, "GREEN");

        if (visionHealth == "RED") {
            state = HealthState.RED;
            MustangNotifications.reportError("RED Error: Vision System");
        } else if (visionHealth == "YELLOW") {
            state = HealthState.YELLOW;
            MustangNotifications.reportWarning("YELLOW Error: Vision System");
        } else {
            state = HealthState.GREEN;
        }
        return state;
    }

    @Override
    public void mustangPeriodic() { 

    }

}