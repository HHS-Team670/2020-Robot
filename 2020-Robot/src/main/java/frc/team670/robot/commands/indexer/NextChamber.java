package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Indexer;

/**
 * Rotates the indexer by 1 chamber over
 */
public class NextChamber extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private int topChamber;

    // private boolean isForward;

    public NextChamber(Indexer indexer) {
        // this.isForward = isForward;
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.YELLOW); // This can work without ToF
    }

    @Override
    public void initialize() {
        topChamber = indexer.getTopChamber();  
    }

    @Override
    public void execute() {
        // if (!indexer.isChamberFull(3)) {
            indexer.run();
        // }
    }

    @Override
    public boolean isFinished() {
        // return indexer.hasReachedTargetPosition();
        boolean shootChamberFull = indexer.isChamberFull(3);
        if (shootChamberFull) {
            Logger.consoleLog("Cannot run this command when shoot chamber is full");
            return true;
        }
        return (topChamber == indexer.getTopChamber() - 1);
        // return (indexer.isChamberFull(bottomChamber + 1));
    }

    public void end() {
        indexer.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}