package frc.team670.robot.commands;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;

import java.util.HashMap;
import java.util.Map;

public class TestCommand extends MustangCommandBase{

    protected boolean ended, exe;

    private Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> healthRequirements;

    public TestCommand(MustangSubsystemBase a, MustangSubsystemBase b, MustangSubsystemBase c){
        addRequirements(a, b, c);
        healthRequirements = new HashMap<MustangSubsystemBase, MustangSubsystemBase.HealthState>();
        healthRequirements.put(a, MustangSubsystemBase.HealthState.GREEN);
        healthRequirements.put(b, MustangSubsystemBase.HealthState.GREEN);
        healthRequirements.put(c, MustangSubsystemBase.HealthState.GREEN);
    }

    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements(){
        return healthRequirements;
    }

    public void setRunning(boolean exe){
        this.exe = exe;
    }

    @Override
    public void end(boolean cancelled){
        if (cancelled) this.ended = false;
    }

    public boolean isRunning(){
        return this.exe;
    }

    public boolean isEnded(){
        return this.ended;
    }
}