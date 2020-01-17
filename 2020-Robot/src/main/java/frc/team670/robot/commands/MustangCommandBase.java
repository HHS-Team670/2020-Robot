package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.team670.robot.subsystems.MustangSubsystemBase;

import java.util.Map;

public abstract class MustangCommandBase extends CommandBase{

    public MustangCommandBase(){
        
    }

    public abstract Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();




}