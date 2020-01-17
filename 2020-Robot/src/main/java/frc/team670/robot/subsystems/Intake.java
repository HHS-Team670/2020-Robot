package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.IRSensor;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;


public class Intake extends SubsystemBase {
    private Compressor comp;
    private Solenoid deployer;
    // private TalonSRX roller;
    private CANSparkMax roller;
    private IRSensor sensor;
    private boolean isDeployed, isRolling;
    private double rollingSpeed;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE); // may need to edit RobotMap to update ports
        comp.setClosedLoopControl(true);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);
        // roller = new TalonSRX(RobotMap.INTAKE_ROLLER);
        roller = new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        sensor = new IRSensor(RobotMap.INTAKE_SENSOR);
    }

    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }
    
    public void setRolling(boolean roll) {
        isRolling = roll;
        if(isRolling) {
            // roller.set(ControlMode.PercentOutput, rollingSpeed);
            roller.set(rollingSpeed);
        }
        else {
            roller.set(0);
        }
    }

    public void setRollingSpeed(double speed) {
        rollingSpeed = speed;
    }

    public boolean getSensor() {
        return sensor.isTriggered();
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public boolean isRolling() {
        return isRolling;
    }
}