package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Intake extends SubsystemBase {
    private Compressor comp;
    private Solenoid deployer;
    // private TalonSRX roller;
    private CANSparkMax roller;
    private boolean isDeployed;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE);
        comp.setClosedLoopControl(true);
        // roller = new TalonSRX(RobotMap.INTAKE_ROLLER);
        roller = new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);
    }

    public void setDeploy(boolean deploy) {
        isDeployed = deploy;
        deployer.set(deploy);
    }
    
    public void rollIntake(double speed) {
        // roller.set(ControlMode.PercentOutput, speed);
        roller.set(speed);
    }

    public boolean isDeployed() {
        return isDeployed;
    }
}