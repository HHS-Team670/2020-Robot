package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Stages 1 ball and spins updraw, then sends it up to shooter when ready
 */
public class SendOneBall extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public SendOneBall(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
    }

    /**
     * Move the indexer to a "staging" position so the updraw can spin up to speed
     */
    @Override
    public void initialize() {
        if (!indexer.updrawIsUpToSpeed()) {
            indexer.rotateToLoadShoot();
        }
    }

    /**
     * Spin the updraw wheels
     */
    @Override
    public void execute() {
        indexer.uptake(); 
    }

    /**
     * After the updraw is ready, rotate so the staged ball can exit the indexer.
     */
    @Override
    public void end(boolean isInterrupted) {
        if (!isInterrupted) {
            indexer.shoot();
        }
    }

    /**
     * @return true when the updraw is up to speed, meaning that we are ready to get
     *         a ball out of the indexer
     */
    @Override
    public boolean isFinished() {
        return indexer.isShootingChamberEmpty();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }

}