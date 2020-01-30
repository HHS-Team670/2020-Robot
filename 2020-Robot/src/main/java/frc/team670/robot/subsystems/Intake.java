package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team670.robot.dataCollection.sensors.IRSensor;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

    private Compressor comp;
	private Solenoid deployer;
    private CANSparkMax rollers;
    private CANEncoder encoder;
    private IRSensor sensor;
    private double rollerSpeed;
    
    // for testing
    private double pValue = 0;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE); // need to edit RobotMap to update ports for irsensor and other stuff
        comp.setClosedLoopControl(true);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_SOLENOID); //define once RobotMap ID given
        rollers = new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new CANEncoder(rollers);
        // sensor = new IRSensor(RobotMap.INTAKE_IRSENSOR);        
    }

    public boolean getSensor() {
        return sensor.isTriggered();
        }
    
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public boolean isRolling() {
        return isRolling;
    }


    @Override
    public HealthState checkHealth() {
        if(encoder.getCountsPerRevolution() != 7)
            return HealthState.GREEN;
        else
            return HealthState.RED;
    }

    public void fr_retract() {
        setDeploy(true);
        if(isDeployed()) {
            setRolling(pValue, true);
        }
    }

    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }

    public void setRolling(boolean roll) {
        isRolling = roll;
        if(isRolling)
            rollers.set(rollerSpeed);
        else
            rollers.set(0);
    }

    public void setRollerSpeed(double percent) {
        rollerSpeed = percent;
    }
    
    }