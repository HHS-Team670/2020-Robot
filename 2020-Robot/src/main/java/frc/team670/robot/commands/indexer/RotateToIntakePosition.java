package frc.team670.robot.commands.indexer;

import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class RotateToIntakePosition extends InstantCommand implements MustangCommand {

    private Indexer indexer;

    public RotateToIntakePosition(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        indexer.prepareToIntake();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        return healthReqs;
    }

}