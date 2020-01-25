package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.Logger;

import java.util.Map;

public class MustangScheduler{

    private Command currentCommand;

    private CommandScheduler scheduler;

    public MustangScheduler(){
        scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command -> check(command));
    }

    public void run(){
        scheduler.run();
    }

    public void cancel(Command... commands){
        scheduler.cancel(commands);
    }

    public Command getCurrentlyScheduled(){
        return this.currentCommand;
    }

    public void cancelAll(){
        scheduler.cancelAll();
    }

    public void schedule(Command command){

    if (command instanceof MustangCommandBase){
        Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> requirements = ((MustangCommandBase)(command)).getHealthRequirements();
        for (MustangSubsystemBase s: requirements.keySet()){
          MustangSubsystemBase .HealthState healthReq = requirements.get(s); 
          if (s.getHealth(false).getId() > healthReq.getId()){
                //DriverStation.reportError(command.getName() + " not run because of health issue! Required health: " + healthReq + ", Actual health: " + s.getHealth(false), false);
                Logger.consoleLog(command.getName() + " not run because of health issue! Required health: " + healthReq + ", Actual health: " + s.getHealth(false));
                command.cancel();
                return;
            }
        }
        this.currentCommand = command;
        scheduler.schedule(command);
        Logger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
        this.currentCommand = null;
      }

    }

    public void check(Command command) throws RuntimeException{
        if (!this.currentCommand.equals(command)){
            throw new RuntimeException("Command was not properly scheduled. Are you using MustangScheduler?");
        }
    }

}