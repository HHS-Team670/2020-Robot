package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

/** 
 * Represents the subsystem that intakes balls from floor
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

    private SparkMAXLite roller;
    private Compressor compressor;
    private Solenoid deployer;
    private boolean isDeployed = false; //TODO: true for testing, change this

    private double INTAKE_ROLLER_SPEED = 1.0; // From testing 2/16
    private double INTAKE_PEAK_CURRENT = 35; // Testing
    private int exceededCurrentLimitCount = 0;

    /**
     * constructor
     */
    public Intake() {
        // Intake roller should be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        roller.setInverted(true);
        roller.setOpenLoopRampRate(1.0);
        compressor = new Compressor(RobotMap.PCMODULE);
        compressor.setClosedLoopControl(true);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);
    }

    /**
     * 
     * @return if intake is rolling
     */
    public boolean isRolling() {
        return roller.get() != 0;
    }

    /**
     * deploy the intake
     * @param isDeployed true to be deployed
     */
    public void deploy(boolean isDeployed) {
        this.isDeployed = isDeployed;
        deployer.set(isDeployed);
    }

    /**
     * 
     * @return if intake is deployed
     */
    public boolean isDeployed() {
        return isDeployed;
    }

    /**
     * runs the intake
     * @param reversed true if intake is to roll backwards
     */
    public void roll(boolean reversed) {
        // if (isDeployed) {
            if (reversed) {
                roller.set(INTAKE_ROLLER_SPEED * -1);
            } else {
                roller.set(INTAKE_ROLLER_SPEED);
            }
        // }
    }

    /**
     * 
     * @return if the intake is jammed
     */
    public boolean isJammed(){
        double intakeCurrent = roller.getOutputCurrent();
        if (intakeCurrent > 0.2){
            if (intakeCurrent >= INTAKE_PEAK_CURRENT) {
                exceededCurrentLimitCount++;
            } else {
                exceededCurrentLimitCount = 0;
            }
            if (exceededCurrentLimitCount >= 1){ // 4 consecutive readings higher than peak
                return true;
            }
        }

        return false;

    }

    /**
     * stops running intake
     */
    public void stop() {
        roller.set(0);
    }

    /**
     * @return GREEN if everything is good.RED if the roller has issues, or the intake isn't deployed but thepneumatics have issues, YELLOW for issues with compressor or deployer
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

}