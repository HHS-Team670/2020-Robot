package frc.team670.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author ctychen, lakshbhambhani
 */
public class Vision extends MustangSubsystemBase{

    private NetworkTableObject keyData;

    private Solenoid cameraLEDs;

    private boolean currentlyUsingVision;

    // The name of the subtable set on the Pi
    public static final String VISION_TABLE_NAME = "SmartDashboard";
    public static final String VISION_RETURN_NETWORK_KEY = "vision_values";
    public static final String VISION_TRIGGER_NETWORK_KEY = "vision-data";

    private double distance, horizontalAngle;

    private long previousTimestamp = 0;

    // These are for sending vision health to dashboard
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable healthTable = instance.getTable("/SmartDashboard");

    // table for vision
    NetworkTable visionTable = instance.getTable(VISION_TABLE_NAME);

    // Vision Constants
    public static final double OUTER_TARGET_CENTER = 249; // centimeters

    public Vision() {
        this(VISION_RETURN_NETWORK_KEY);
    }

    private Vision(String key) {
        keyData = new NetworkTableObject(key);
        this.currentlyUsingVision = false;
        cameraLEDs = new Solenoid(RobotMap.PCMODULE, RobotMap.VISION_LED_PCM);
        SmartDashboard.putBoolean("LEDs on", false);
    }

    /**
     * 
     * @return the horizontal angle between the camera-forward to the robot-target line
     */
    public void getCameraToTargetInfo() {
        try{
            double IMAGE_WIDTH = keyData.getEntry(2);
            double IMAGE_HEIGHT = keyData.getEntry(3);
    
            double xPixel = keyData.getEntry(0);
            double yPixel = keyData.getEntry(1);
    
            double nX = xPixel - IMAGE_WIDTH/2;
            double nY = yPixel - IMAGE_HEIGHT/2;

            horizontalAngle = nX/(IMAGE_WIDTH/2) * (RobotConstants.kHorizontalFOV/2);
    
            // double x = RobotConstants.kVPW/2 * nX;
            // double y = RobotConstants.kVPH/2 * nY;
    

            SmartDashboard.putNumber("nX", nX);
            SmartDashboard.putNumber("xPixel", xPixel);

            SmartDashboard.putNumber("imageWidth", IMAGE_WIDTH);

            // horizontalAngle = Math.atan2(1,x);
            // double verticalAngle = Math.atan2(1,y);
    
            // distance = (FieldConstants.TARGET_CENTER_HEIGHT-RobotConstants.TURRET_CAMERA_HEIGHT) / Math.tan(RobotConstants.TILT_ANGLE+verticalAngle);
        }
        catch (Exception e){
            MustangNotifications.reportWarning(e.getMessage());
        }
        
    }

    public double getAngleToTarget(){
        return horizontalAngle;
    }

    public double getDistanceToTargetM(){
        return distance;
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    public double getDistanceToTargetInches() {
        return getDistanceToTargetCm() / 2.54;
    }

    /**
     * 
     * @return distance, in cm, from the camera to the target
     */
    public double getDistanceToTargetCm() {
        return getDistanceToTargetM() * 100;
    }

    private class NetworkTableObject {

        private NetworkTableEntry entry;
        private String key;

        /**
         * The key of the NetworkTableEntry that this Object will be attached to.
         */
        public NetworkTableObject(String key) {
            entry = instance.getEntry(key);
            visionTable.addEntryListener(key, (table2, key2, entry, value, flags) -> {
                this.entry = entry;
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
            this.key = key;
        }

        /**
         * Gets the value of the entry, this will be returned as a Double Array. Make
         * sure to have a try/catch block for the Exception
         * 
         * @return A Double Array with the value or values last received from the pi.
         *         Fills with Vision Error Code if it can't get it. Format is [angle to
         *         target, angle for straight on to target, distance from target]
         */
        public double[] getValue() {
            if (entry.getType().equals(NetworkTableType.kDoubleArray)) {
                return entry.getDoubleArray(new double[] { RobotConstants.VISION_ERROR_CODE,
                        RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE });
            }
            return new double[] { RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE,
                    RobotConstants.VISION_ERROR_CODE };
        }

        /**
         * @return the entry at the given index in the array at the NetworkTable entry
         *         that corresponds to this object's key.
         */
        public double getEntry(int index) {
            if (entry == null) {
                return RobotConstants.VISION_ERROR_CODE; // If no data found, returns VISION_ERROR_CODE
            }
            double result = getValue()[index];
            return result;
        }

        public NetworkTableEntry getEntry() {
            return entry;
        }

    }

    /**
     * Sets whether to use vision
     * 
     * @param enabled true for vision, false for no vision
     */
    public void enableVision(boolean enabled) {
        NetworkTableEntry visionHealth = healthTable.getEntry("vision");
        if (enabled) {
            this.currentlyUsingVision = true;
            visionHealth.forceSetString("green");
        } else {
            this.currentlyUsingVision = false;
            visionHealth.forceSetString("red");
        }
    }

    /**
     * @return whether or not vision is currently running/in use
     */
    public boolean isVisionEnabled(){
        return this.currentlyUsingVision;
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
        // if(keyData.getEntry(2) != previousTimestamp){
        //     getCameraToTargetInfo();
        //     previousTimestamp = (long)keyData.getEntry(2);
        // }
        getCameraToTargetInfo();
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Angle", horizontalAngle);
    }

}