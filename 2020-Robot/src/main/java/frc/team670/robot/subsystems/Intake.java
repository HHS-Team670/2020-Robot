package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

    private SparkMAXLite roller;
    private Compressor compressor;
    private Solenoid deployer;
    private boolean isDeployed;

    private double INTAKE_ROLLER_SPEED = 0.6; // From testing 2/16

    public Intake() {
        // Intake roller should be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        roller.setInverted(true);
        
        compressor = new Compressor(RobotMap.PCMODULE);
        compressor.setClosedLoopControl(true);
        SmartDashboard.putNumber("Intake Speed", 0.0);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);
    }

    public boolean isRolling() {
        return roller.get() != 0;
    }

    public void deploy(boolean isDeployed) {
        this.isDeployed = isDeployed;
        deployer.set(isDeployed);
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public void roll(boolean reversed) {
        if (isDeployed) {
            if (reversed) {
                roller.set(INTAKE_ROLLER_SPEED * -1);
            } else {
                roller.set(INTAKE_ROLLER_SPEED);
            }
        }
    }

    public void stop() {
        roller.set(0);
    }

    /**
     * @return RED if the roller has issues, or the intake isn't deployed but the
     *         pneumatics have issues
     */
    @Override
    public HealthState checkHealth() {
        if (roller == null || isSparkMaxErrored(roller)) {
            return HealthState.RED;
        }
        if (compressor == null || deployer == null) {
            if (isDeployed) {
                // If there's a problem with only pneumatics but the intake is already
                // deployed, we can still roll the intake
                return HealthState.YELLOW;
            }
            // If it isn't deployed and we have a pneumatics issue, we can't do anything
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
    }

    public void test() {
        roller.set(SmartDashboard.getNumber("Intake Speed", 0.0));
    }

}