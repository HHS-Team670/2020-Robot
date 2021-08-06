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
public class ShootBall extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    // private int bottomChamber;
    private int totalNumBalls;
    // private boolean isForward;

    public ShootBall(Indexer indexer) {
        // this.isForward = isForward;
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.YELLOW); // This can work without ToF
    }

    @Override
    public void initialize() {
        Logger.consoleLog("Preparing to shoot one ball");
        indexer.updraw(false);
        totalNumBalls = indexer.totalNumBalls();
        // bottomChamber = indexer.getBottomChamber();
    }

    @Override 
    public void execute() {
        if (indexer.updrawIsUpToSpeed()) {
            indexer.run();
        }
    }

    @Override
    public boolean isFinished() {
        if (!indexer.ballInChamber(3)) {
            Logger.consoleLog("Cannot run this command when ball is not in shooting chamber.");
            return true;
        }
        // return indexer.hasReachedTargetPosition();
        // return bottomChamber == indexer.getBottomChamber() - 1 || indexer.totalNumBalls() == 0;
        return (totalNumBalls == indexer.totalNumBalls() - 1);
    }

    public void end() {
        indexer.stopUpdraw();
        indexer.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}