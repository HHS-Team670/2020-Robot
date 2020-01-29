package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team670.robot.dataCollection.sensors.IRSensor;


public class Intake extends MustangSubsystemBase {

    private Compressor comp;
	private Solenoid deployer;
    private TalonSRX rollers;
    private IRSensor sensor;
    private double rollerSpeed;
	private boolean isDeployed, isRolling;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE); // may need to edit RobotMap to update ports
        comp.setClosedLoopControl(true);
        // deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_SOLENOID); define once RobotMap ID given
        rollers = new TalonSRX(RobotMap.INTAKE_ROLLER);
        // sensor = new IRSensor(RobotMap.INTAKE_IRSENSOR);
    }

    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }

    public void setRolling(boolean roll) {
        isRolling = roll;
        if(isRolling)
            rollers.set(ControlMode.PercentOutput,rollerSpeed);
        else
            rollers.set(ControlMode.PercentOutput,0);
    }

    public boolean getSensor() {
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

    /** 
     * TODO: These should be moved to commands if they haven't been already.
    */
    // // deploy, retract, and roll commands(speeds need to be set)
    // public void a_deploy() { // uses ir sensor to detect ball to deploy then roll/spin motorz(autonomous
    //                          // deploy)
    //     if (!isDeployed() && getSensor()) {
    //         setDeploy(true);
    //         if (isDeployed()) {
    //             setRolling(changingRollingSpeed, true);
    //         }
    //     }
    // }

    // public void m_deploy() {
    //     if (!isDeployed()) {
    //         setDeploy(true);
    //         if (isDeployed()) {
    //             setRolling(changingRollingSpeed, true);
    //         }
    //     }
    // }

    // public void retract() {
    //     setDeploy(true);
    //     if (isDeployed()) {
    //         setRolling(changingRollingSpeed, true);
    //     }
    // }

    // public void unjam() { // use if thing jammed 
    //     if (!isDeployed()) {
    //         setDeploy(true);
    //     }

    //     setRolling(-changingRollingSpeed, true);

    //     // TODO 
    // }

}