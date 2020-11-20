package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Toggles the updraw state between stopped and moving
 */
public class ToggleUpdraw extends InstantCommand implements MustangCommand {

    private Indexer indexer;
    
    /**
     * Initializes this command from the given indexer
     * @param indexer the indexer of the robot
     */
    public ToggleUpdraw(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        indexer.toggleUpdraw();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return new HashMap<MustangSubsystemBase, HealthState>();
    }

    
}