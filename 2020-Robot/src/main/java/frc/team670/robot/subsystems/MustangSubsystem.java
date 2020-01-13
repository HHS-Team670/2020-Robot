package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team670.robot.RobotContainer;

public abstract class MustangSubsystem extends SubsystemBase{

    public MustangSubsystem(){
        RobotContainer.addSubsystem(this);
    }

    public abstract HealthState checkHealth();

    public enum HealthState{
        HEALTHY, WARNING, CRITICAL;
    }

    public void initDefaultCommand(Command command){
        CommandScheduler.getInstance().setDefaultCommand(this, command);
    }

}