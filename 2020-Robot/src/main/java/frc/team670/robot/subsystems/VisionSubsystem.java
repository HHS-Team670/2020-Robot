package frc.team670.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author lakshbhambhani, katia, pallavi
 */
public class VisionSubsystem extends MustangSubsystemBase {

    private Solenoid cameraLEDs;

    private final String VISION_TRIGGER_KEY = "vision-enable";
    private final String VISION_VALUES_KEY = "vision-values";

    private final String VISION_MIN_HSV_KEY = "vision-minHSV";
    private final String VISION_MAX_HSV_KEY = "vision-maxHSV";
    private final String VISION_SHAPE_KEY = "vision-shape";

    /**
     * Used to create a mustangCoprocessor object based on the key of the table that returns the values from vision processing
     */
    private VisionSubsystem(String key, int[] minHSV, int[] maxHSV, int[] shape) {
        cameraLEDs = new Solenoid(RobotMap.PCMODULE, RobotMap.VISION_LED_PCM);
        SmartDashboard.putBoolean("LEDs on", false);
        
        SmartDashboard.putNumberArray(VISION_MIN_HSV_KEY, minHSV);
        SmartDashboard.putNumberArray(VISION_MAX_HSV_KEY, maxHSV);
        SmardDashboard.putNumberArray(VISION_SHAPE_KEY, shape);
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
        SmartDashboard.getNumberArray(VISION_VALUES_KEY, [-1,-1]);
    }

    /**
     * Used to clear last values present on the table
     */
    public void clearLastValues(){
        SmartDashboard.putNumberArray(VISION_VALUES_KEY, [-1, -1]);
    }

    /**
     * 
     * @return the horizontal angle between the camera-forward to the robot-target line
     */
    public double getAngleToTarget() {
        return keyData.getEntry(0);
    }

    /**
     * 
     * @return the horizontal angle between the target-forward and the
     *         robot-target line
     */
    public double getAngleToTargetPerpendicular() {
        return keyData.getEntry(1);
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    public double getDistanceToTargetInches() {
        return keyData.getEntry(2);
    }

    /**
     * 
     * @return distance, in cm, from the camera to the target
     */
    public double getDistanceToTargetCm() {
        return getDistanceToTargetInches() * 2.54;
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
    // public void enableVision(boolean enabled) {

    //     NetworkTableEntry visionHealth = healthTable.getEntry("vision");
    //     if (enabled) {
    //         this.currentlyUsingVision = true;
    //         visionHealth.forceSetString("green");
    //     } else {
    //         this.currentlyUsingVision = false;
    //         visionHealth.forceSetString("red");
    //     }
    // }

    /**
     * @return whether or not vision is currently running/in use
     */
    public boolean isVisionEnabled(){
        return this.currentlyUsingVision;
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
        HealthState state = HealthState.GREEN;

        visionHealth = SmartDashboard.getString(visionHealth);

        if (visionKey == "RED") {
            state = HealthState.RED;
            MustangNotifications.reportError("RED Error: %s", visionKey);
        } else {
            state = HealthState.GREEN;
        }
        return state;
    }

    @Override
    public void mustangPeriodic() { 
        // TODO Auto-generated method stub
        
    }

}