package frc.team670.robot.subsystems;

import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.constants.RobotMap;

import com.revrobotics.CANError;

import edu.wpi.first.wpilibj.Timer;

/**
 * Represents the conveyor subsystem that sends balls from intake to indexer
 * 
 * @author ctychen
 */
public class Conveyor extends MustangSubsystemBase {

    private SparkMAXLite roller;

    private double CONVEYOR_SPEED = 0.75; // % output from testing 2/16.

    /**
     * constructor
     */
    public Conveyor() {
        // Conveyor motor should not be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CONVEYOR_ROLLER, Motor_Type.NEO_550);
    }

    /**
     * runs the conveyer
     * @param reversed True to run the conveyor in reverse
     */
    public void run(boolean reversed) {
        if (reversed) {
            roller.set(CONVEYOR_SPEED * -1);
        } else {
            roller.set(CONVEYOR_SPEED);
        }
    }

    // public void setRunTimed(double speed, double timeSecs) {
    //     Timer timer = new Timer();
    //     timer.start();
    //     if (timer.hasElapsed(timeSecs)) {
    //         roller.set(speed);
    //     }
    //     timer.stop();
    //     stop();
    // }

    /**
     * stops conveyer
     */
    public void stop() {
        roller.set(0);
    }

    /**
     * @return GREEN if everything is fine, RED if there are issues with the roller
     */
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