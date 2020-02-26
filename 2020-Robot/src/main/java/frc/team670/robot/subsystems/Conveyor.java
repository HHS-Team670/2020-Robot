package frc.team670.robot.subsystems;

import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.constants.RobotMap;

import com.revrobotics.CANError;

import edu.wpi.first.wpilibj.Timer;

public class Conveyor extends MustangSubsystemBase {

    private SparkMAXLite roller;
    private Timer timer;

    private double CONVEYOR_SPEED = 0.75; // % output from testing 2/16.

    public Conveyor() {
        // Conveyor motor should not be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CONVEYOR_ROLLER, Motor_Type.NEO_550);
        timer = new Timer();
        timer.start();
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

    public void setRunTimed(double speed, double timeSecs) {
        if (timer.hasPeriodPassed(timeSecs)) {
            roller.set(speed);
        }
        stop();
    }

    public void stop() {
        roller.set(0);
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