package frc.team670.robot.subsystems;

import com.revrobotics.CANError;

//import frc.team670.mustanglib.dataCollection.sensors.Multiplexer;
import frc.team670.mustanglib.dataCollection.sensors.TimeOfFlightSensor;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

public class Conveyor extends MustangSubsystemBase {

    private SparkMAXLite roller;

    private double CONVEYOR_SPEED = 0.75; // % output from testing 2/16.

    private TimeOfFlightSensor entranceSensor;
    
    //private Multiplexer multiplexer;

    public Conveyor() {
        // Conveyor motor should not be inverted
        // this.multiplexer = multiplexer;
        // entranceSensor = new TimeOfFlightSensor(RobotMap.INDEXER_MUL_PORT, 0, 20);
        // multiplexer.attachSensor(entranceSensor);
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CONVEYOR_ROLLER, Motor_Type.NEO_550);
    }

    /**
     * 
     * @param reversed True to run the conveyor in reverse
     */
    public void run(boolean reversed) {
        if (reversed) {
            roller.set(CONVEYOR_SPEED * -1);
        } else {
            roller.set(CONVEYOR_SPEED);
        }
    }

    public void stop() {
        roller.set(0);
    }

    public boolean isBallInConveyor(){
        // return multiplexer.getSensors().get(0).getDistance() < RobotConstants.MIN_BALL_DETECTED_WIDTH;
        return false;
    }

    @Override
    public HealthState checkHealth() {
        if (roller.getLastError() != null && roller.getLastError() != CANError.kOk)
            return HealthState.RED;
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

}