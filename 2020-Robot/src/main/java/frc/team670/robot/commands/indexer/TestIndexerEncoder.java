package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Run this only once to get a constant for the absolute encoder, before everything
 * or if the indexer was taken apart.
 */
public class TestIndexerEncoder extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Indexer indexer;

    public TestIndexerEncoder(Indexer indexer) {
        this.indexer = indexer;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.YELLOW);
        addRequirements(indexer);
    }

    /**
     * Use this to find the constant for what the absolute encoder reads at our position 0
     */
    @Override
    public void execute() {
        SmartDashboard.putNumber("Indexer absolute encoder rotations", indexer.getAbsoluteEncoderRotations());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}