package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

    private SparkMAXLite roller;
    private Compressor compressor;
    private Solenoid deployer;
    private boolean isDeployed;

    public Intake() {
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        compressor = new Compressor(RobotMap.INTAKE_COMPRESSOR);
        compressor.setClosedLoopControl(true);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);
    }

    public boolean isRolling() {
        return roller.get() != 0;
    }

    public void deploy(boolean isDeployed) {
        this.isDeployed = isDeployed;
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public void roll(double speed) {
        if (isDeployed) {
            roller.set(speed);
        }
    }

    @Override
    public HealthState checkHealth() {
        if (roller != null && compressor != null && deployer != null) {
            return HealthState.GREEN;
        } else if (roller != null && isDeployed) {
            return HealthState.YELLOW;
        } else {
            return HealthState.RED;
        }
    }

    @Override
    public void mustangPeriodic() {
    }

}