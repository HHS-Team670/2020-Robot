package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import frc.team670.robot.subsystems.MustangSubsystemBase;

import java.util.Map;

public class MustangScheduler{



    public void run(){
        CommandScheduler.getInstance().run();
    }

    public void cancel(Command... commands){
        CommandScheduler.getInstance().cancel(commands);
    }


    public void cancelAll(){
        CommandScheduler.getInstance().cancelAll();
    }

    public void schedule(Command command){

    if (command instanceof MustangCommandBase){
        Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> requirements = ((MustangCommandBase)(command)).getHealthRequirements();
        for (MustangSubsystemBase s: requirements.keySet()){
          if (s.getHealth(false).getId() > requirements.get(s).getId()){
                DriverStation.reportError(command.getName() + " not run because of health issue! ", false);

          }
        }
      }

    }



}