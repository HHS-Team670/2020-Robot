package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.team670.robot.subsystems.MustangSubsystem;

import java.util.Map;
import static java.util.Map.entry;
import java.util.ArrayList;

public abstract class MustangCommandBase extends CommandBase {

    

    public MustangCommandBase(){
        
    }

    public Map<Subsystem, MustangSubsystem.HealthState> getHealthRequirements(){

    }




}