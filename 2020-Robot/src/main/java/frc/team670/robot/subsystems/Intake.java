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

    private double changingRollingSpeed = 0.8;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE); // may need to edit RobotMap to update ports
        comp.setClosedLoopControl(true);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);
        // roller = new TalonSRX(RobotMap.INTAKE_ROLLER);
        roller = new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        sensor = new IRSensor(RobotMap.INTAKE_SENSOR);
    }
    
    // basic fetch and set commands
    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }
    
    public void setRolling(double speed, boolean roll) {
        isRolling = roll;
        rollingSpeed = speed;
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

    // autonomous commands
    public void a_deploy() { // uses ir sensor to detect ball to deploy then roll/spin motorz(autonomous deploy)
        if(!isDeployed() && getSensor()) {
            setDeploy(true);
            if(isDeployed()) {
                setRollingSpeed(changingRollingSpeed); // unsure if this is the correct motor speed
                setRolling(true);
            }
        }
    }

    public void m_deploy() {
        if(!isDeployed()) {
            
        }
    }
}