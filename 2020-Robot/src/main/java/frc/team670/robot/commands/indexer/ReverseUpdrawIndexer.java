package frc.team670.robot.commands.indexer;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReverseUpdrawIndexer extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public ReverseUpdrawIndexer(Indexer indexer) {
        this.indexer = indexer;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        indexer.rotateToEmptyAtTop();
    }

    @Override
    public void execute() {
        // Runs updraw in reverse
        indexer.updraw(true);
    }

    @Override
    public boolean isFinished() {
        // Top chamber has a ball in it
        return !indexer.isShootingChamberEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            indexer.stopUpdraw();
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}