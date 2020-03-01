package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

/**
 * Stages 1 ball in preparation to shoot, and spins updraw
 * 
 * @author ctychen
 */
public class EmptyRevolver extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public EmptyRevolver(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
    }

    /**
     * Move the indexer to a "staging" position so the updraw can spin up to speed
     */
    @Override
    public void initialize() {
        indexer.updraw(false);
    }

    /**
     * Spin the updraw wheels
     */
    @Override
    public void execute() {
        indexer.updraw(false);
        if (indexer.updrawIsUpToSpeed()) {
            indexer.spinRevolver();            
        }
    }

    /**
     * @return true when the indexer has rotated to the position for staging the
     *         ball
     */
    @Override
    public boolean isFinished() {
        return indexer.hasReachedTargetPosition();

    }

    @Override
    public void end(boolean interrupted){
        indexer.stopUpdraw();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }

}