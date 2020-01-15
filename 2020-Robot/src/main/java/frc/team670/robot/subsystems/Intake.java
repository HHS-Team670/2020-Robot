package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private Compressor comp;
    private Solenoid deployer;
    private TalonSRX axleMotor;
    public boolean isDeployed, isSpinning;

    public Intake() {
        comp = new Compressor(RobotMap.PCMODULE);
        comp.setClosedLoopControl(true);
        axleMotor = new TalonSRX(RobotMap.INTAKE);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.DEPLOYER);
    }

    public void spinIntake(double speed) {
        axleMotor.set(ControlMode.PercentOutput, speed);
    }

    public void deployIntake(boolean deploy) {
        isDeployed = deploy;
        deployer.set();
    }
}