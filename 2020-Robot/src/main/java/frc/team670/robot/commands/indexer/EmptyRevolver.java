package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;

/**
 * Empties the revolver by doing 1 full spin (360 degrees as opposed to 5 chambers 1 at a time)
 * 
 * @author ctychen
 */
public class EmptyRevolver extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private boolean indexerSpun = false;
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
        Logger.consoleLog("Preparing to empty revolver system");
        indexer.clearSetpoint(); // Keep piston from deploying at beginning
        indexer.updraw(false);
    }

    /**
     * Spin the updraw wheels
     */
    @Override
    public void execute() {
        indexer.updraw(false);
        if (indexer.updrawIsUpToSpeed() && !indexerSpun) {
            indexer.spinRevolver();   
            indexerSpun = true;         
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
        Logger.consoleLog("Revolver system emptied");
        indexer.stopUpdraw();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }

}