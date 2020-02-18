package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * One option for unjamming the indexer is to "wiggle" it back and forth until
 * unjammed, which is implemented here. 
 * TODO: Actually experiment with the
 * indexer, see what unjams it, and change this if needed -- there's probably other ways
 */
public class UnjamIndexer extends CommandBase implements MustangCommand {

    private Indexer indexer;

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private long lastSwitchedDirection;
    private int direction;

    public UnjamIndexer(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        if (indexer.getSpeed() < 0) {
            direction = 1;
        } else {
            direction = -1;
        }
        lastSwitchedDirection = System.currentTimeMillis();
    }

    // "Wiggle" the indexer back and forth a bit to unjam?
    @Override
    public void execute() {
        // Using time because using distance rotated wouldn't be reliable
        if (System.currentTimeMillis() - lastSwitchedDirection < 100) {
            indexer.setTargetAngleInDegrees(indexer.getCurrentAngleInDegrees() + 10 * direction);
        } else {
            direction = -direction;
            lastSwitchedDirection = System.currentTimeMillis();
        }

    }

    @Override
    public boolean isFinished() {
        // true when indexer rotator current is back to normal (unjammed)
        return !indexer.isIndexerJammed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}