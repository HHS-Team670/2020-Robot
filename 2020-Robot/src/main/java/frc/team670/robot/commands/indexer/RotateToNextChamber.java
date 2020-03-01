package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Rotates the indexer by 1 chamber over
 */
public class RotateToNextChamber extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private boolean isForward;

    public RotateToNextChamber(Indexer indexer, boolean isForward) {
        this.isForward = isForward;
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.YELLOW); // This can work without ToF
    }

    @Override
    public void initialize() {
        if(isForward){
            indexer.rotateToNextChamber();
        }
        else{
            indexer.rotateToPreviousChamber();
        }
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