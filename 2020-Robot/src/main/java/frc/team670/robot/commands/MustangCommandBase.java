package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.subsystems.MustangSubsystemBase;

import java.util.Map;

/**
 * Represents a robot action with defined health requirements for every subsystem it uses.
 * 
 * @author ctychen
 */
public interface MustangCommandBase{

    /**
     * @return A Map containing the minimum health condition for each subsystem that this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

}