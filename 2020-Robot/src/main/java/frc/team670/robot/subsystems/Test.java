package frc.team670.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;

public class Test extends SparkMaxRotatingSubsystem {

    public Test() {
        super(RobotMap.SPARK_LEFT_MOTOR_1, SmartDashboard.getNumber("P", 0.001), 0, 0, 0, 1000000, -1000000, false, 30, 5, 0);
    }

    @Override
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void moveByPercentOutput(double output) {
        // TODO Auto-generated method stub

    }

    @Override
    public double getAngleInDegrees() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }

   
}