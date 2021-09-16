package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.team670.mustanglib.dataCollection.sensors.TimeOfFlightSensor;
import frc.team670.robot.constants.RobotConstants;

public class BallSensor extends TimeOfFlightSensor {

    final static int threshold = 5; // in mm

    public BallSensor(Port port, int sensorAddress) {
        super(port, true, sensorAddress);
    }

    public boolean isBallDetected() {
        return Math.abs(RobotConstants.SENSOR_TO_BALL - super.getDistance()) <= threshold;
    }

}