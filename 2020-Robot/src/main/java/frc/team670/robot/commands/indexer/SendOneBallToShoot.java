package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Rotates indexer to shooting position and kicks ball up to shooter.
 * 
 * @author ctychen
 */
public class SendOneBallToShoot extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public SendOneBallToShoot(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        if (indexer.updrawIsUpToSpeed()) {
            indexer.shoot();
        }
    }

    @Override
    public void execute() {
        indexer.updraw();
    }

    /**
     * @return true when the top chamber of the indexer is empty (ball has been
     *         updrawed)
     */
    @Override
    public boolean isFinished() {
        return indexer.isShootingChamberEmpty();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}