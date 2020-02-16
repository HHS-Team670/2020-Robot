package frc.team670.robot.commands.indexer;

import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class RotateToIntakePosition extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public RotateToIntakePosition(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        indexer.prepareToIntake();
    }

    @Override
    public boolean isFinished() {
        return indexer.hasReachedTargetPosition();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}