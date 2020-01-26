package frc.team670.robot.commands;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.Logger;

import java.util.HashMap;
import java.util.Map;

public class TestCommand extends MustangCommandBase{

    protected boolean ended, exe, cancelled;

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

    @Override
    public void initialize() {
        this.exe = true;
        Logger.consoleLog("Initialized");
    }

    @Override
    public void end(boolean cancelled){
        if (cancelled) this.ended = true; //end is never called? just cancel, which doesn't call end
    }

    public void cancel() {
        this.cancelled = true;
    }

    public boolean isCancelled() {
        return this.cancelled;
    }

    public boolean isRunning(){
        return this.exe;
    }

    public boolean isEnded(){
        return this.ended;
    }
}