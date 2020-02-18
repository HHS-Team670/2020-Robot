package frc.team670.robot.dataCollection;

import java.util.HashMap;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 */
public class MustangCoprocessor {

    private NetworkTableObject keyData;

    private Solenoid cameraLEDs;

    // The name of the subtable set on the Pi
    private static final String TABLE_NAME = "SmartDashboard";
    private static final String NETWORK_KEY = "reflect_tape_vision_data";

    // Vision Constants
    public static final double OUTER_TARGET_CENTER = 249; // centimeters

    private double cameraHorizontalOffset; // centimeters
    private double verticalCameraOffsetAngle; // degrees
    private double cameraHeight; // centimeters

    public MustangCoprocessor() {
        this(NETWORK_KEY);
    }

    private MustangCoprocessor(String key) {
        keyData = new NetworkTableObject(key);
        cameraLEDs = new Solenoid(RobotMap.PCMODULE, RobotMap.VISION_LED_PCM);
        SmartDashboard.putBoolean("LEDs on", false);
    }

    /**
     * 
     * @return angle from camera to the target, in degrees
     */
    public double getAngleToTarget() {
        return keyData.getEntry(0);
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
            NetworkTableInstance instance = NetworkTableInstance.getDefault();
            NetworkTable table = instance.getTable(TABLE_NAME);
            entry = instance.getEntry(key);
            table.addEntryListener(key, (table2, key2, entry, value, flags) -> {
                this.entry = entry;
                System.out.println("Changed Value");
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

    public void turnOnLEDs() {
        cameraLEDs.set(true);
    }

    public void turnOffLEDs() {
        cameraLEDs.set(false);
    }

    public void testLEDS() {
        cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
    }

}