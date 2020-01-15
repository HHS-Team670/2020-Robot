package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.RobotContainer;

import java.util.Map;
import static java.util.Map.entry;
import java.util.ArrayList;

public abstract class MustangSubsystem extends SubsystemBase{

    private HealthState lastHealthState;

    private ArrayList<MustangCommand> greenCommands;
    private ArrayList<MustangCommand> yellowCommands;
    private ArrayList<MustangCommand> redCommands;

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

    public void initDefaultCommand(MustangCommand command){
        CommandScheduler.getInstance().setDefaultCommand(this, command);
    }

    public void addCommandForState(HealthState state, MustangCommand command){
        if (state == HealthState.GREEN) greenCommands.add(command);
        if (state == HealthState.YELLOW) yellowCommands.add(command);
        if (state == HealthState.RED) redCommands.add(command);
    }

  

}