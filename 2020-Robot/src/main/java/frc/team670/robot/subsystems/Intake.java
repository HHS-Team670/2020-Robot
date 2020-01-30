package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team670.robot.dataCollection.sensors.IRSensor;

public class Intake extends MustangSubsystemBase {

    private Compressor comp;
	private Solenoid deployer;
    private CANSparkMax roller;
    private IRSensor sensor;
    private double rollerSpeed;
	private boolean isDeployed, isRolling;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE); // may need to edit RobotMap to update ports
        comp.setClosedLoopControl(true);
        // deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_SOLENOID); define once RobotMap ID given
        roller = new CANSparkMax(RobotMap.INTAKE_ROLLER, MotorType.kBrushless); //assuming use of a NEO550
        // sensor = new IRSensor(RobotMap.INTAKE_IRSENSOR); //TBD if we'll be using this, and what sensor we'll use
    }

    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }

    public void setRolling(boolean roll) {
        isRolling = roll;
        if(isRolling)
            roller.set(rollerSpeed);
        else
            roller.set(0);
    }

    public boolean isSensorTriggered() {
        return sensor.isTriggered();
    }

    public void setRollerSpeed(double percent) {
        rollerSpeed = percent;
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public boolean isRolling() {
        return isRolling;
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub, add to this
        return null;
    }

}