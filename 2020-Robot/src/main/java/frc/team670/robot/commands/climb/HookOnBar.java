package frc.team670.robot.commands.climb;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.climber.Climber;

/**
 * Moves the climber slowly down until it hooks on the generator bar. 
 */
public class HookOnBar extends CommandBase implements MustangCommand {

    private Climber climber;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean hooked;

    public HookOnBar(Climber climber) {
        this.climber = climber;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
        hooked = false;
    }

    @Override
    public void initialize() {
        climber.setPower(0);
    }

    @Override
    public void execute() {
        if (!hooked) {
            climber.hookOnBar();
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isHookedOnBar();
    }

    @Override
    public void end(boolean interrupted){
        this.hooked = true;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}