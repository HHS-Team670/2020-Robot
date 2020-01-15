package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.RobotContainer;


public abstract class MustangSubsystem extends SubsystemBase{

    private HealthState lastHealthState;

    public MustangSubsystem(){
        RobotContainer.addSubsystem(this);
    }

    
    public enum HealthState{
        GREEN, YELLOW, RED;
    }

    public HealthState getHealth(){
        return this.lastHealthState;
    }

    protected abstract HealthState checkHealth();

    public void setHealthState(){
        lastHealthState = checkHealth();
    }

    public void initDefaultCommand(MustangCommandBase command){
        CommandScheduler.getInstance().setDefaultCommand(this, command);
    }
  

}