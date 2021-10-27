package frc.team670.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.mustanglib.utils.math.interpolable.PolynomialRegression;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author ctychen, lakshbhambhani
 */
public class Vision extends MustangSubsystemBase{

    private NetworkTableObject keyData;

    private Solenoid cameraLEDs;

    // The name of the subtable set on the Pi
    public static final String VISION_TABLE_NAME = "SmartDashboard";
    public static final String VISION_RETURN_NETWORK_KEY = "vision_values";
    public static final String VISION_TRIGGER_NETWORK_KEY = "vision-data";

    private double distance, horizontalAngle;

    private long previousTimestamp = 0;

    // These are for sending vision health to dashboard
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();

    // table for vision
    NetworkTable visionTable = instance.getTable(VISION_TABLE_NAME);

    // Vision Constants
    public static final double OUTER_TARGET_CENTER = 249; // centimeters

    private static final double[] measuredDistancesMeters = {
        3.05, //10ft 
        3.66, 
        4.27,
        4.88, 
        5.49, 
        6.1, 
        6.71, 
        7.32 //24ft
      };
    
      private static final double[] measuredContourArea = {
        4365, 
        3526, 
        2590, 
        2046, 
        1593, 
        1325, 
        1104, 
        924
      };
    
      private static final PolynomialRegression distanceRegression = new PolynomialRegression(measuredDistancesMeters, measuredContourArea, 4);

    public Vision() {
        this(VISION_RETURN_NETWORK_KEY);
    }

    private Vision(String key) {
        keyData = new NetworkTableObject(key);
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
    
            double xPixel = keyData.getEntry(0);
    
            double nX = xPixel - IMAGE_WIDTH/2;

            horizontalAngle = nX/(IMAGE_WIDTH/2) * (RobotConstants.kHorizontalFOV/2);

            distance = distanceRegression.predict(keyData.getEntry(4));
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
        // if(keyData.getEntry(1) == RobotConstants.VISION_ERROR_CODE){
        //     return HealthState.RED;
        // }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // if(keyData.getEntry(5) != previousTimestamp){
        //     getCameraToTargetInfo();
        //     previousTimestamp = (long)keyData.getEntry(5);
        // }
        getCameraToTargetInfo();
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Angle", horizontalAngle);
    }

}