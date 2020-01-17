package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team670.robot.dataCollection.sensors.IRSensor;


public class Intake extends SubsystemBase {
    private Compressor comp;
	private Solenoid deployer;
    private TalonSRX rollers;
    private IRSensor irSensor;
    private double rollerSpeed;
	private boolean isDeployed, isRolling;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE); // may need to edit RobotMap to update ports
        comp.setClosedLoopControl(true);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_SOLENOID);
        rollers = new TalonSRX(RobotMap.INTAKE_ROLLERS);
        irSensor = new IRSensor(RobotMap.INTAKE_IRSENSOR);
    }

    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }

    public void setRolling(boolean roll) {
        isRolling = roll;
        if(isRolling)
            rollers.set(ControlMode.PercentOutput, rollerSpeed);
        else
            rollers.set(ControlMode.PercentOutput, 0);
    }

    public boolean getSensor() { // check if irsensor detects anything(needs to be modified in future for certain distance or such)
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
}