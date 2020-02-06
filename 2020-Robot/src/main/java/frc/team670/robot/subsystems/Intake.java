package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.IRSensor;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.MotorConfig;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

    private CANSparkMax roller;
    private Compressor compressor;
    private Solenoid deployer;
    private int currentLimit;
    private boolean isDeployed;
    
    public Intake() {

        roller = new CANSparkMax(RobotMap.INTAKE_ROLLER, MotorType.kBrushless);
        compressor = new Compressor(RobotMap.INTAKE_COMPRESSOR);
        compressor.setClosedLoopControl(true);
		deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);

        currentLimit = 20;
        roller.setSmartCurrentLimit(currentLimit);

    }
    public boolean isRolling() {
        return roller.get() != 0;
    }

	public void deploy(boolean isDeployed)
	{
        deployer.set(isDeployed);
        isDeployed = true;
    }
    
    public boolean isDeployed() {
        return isDeployed;
    }

    public void roll(double speed) {
        roller.set(speed);
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