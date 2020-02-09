package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Resets the revolver to the 0 position, defined by a magnet
 */
public class ZeroRevolver extends InstantCommand implements MustangCommand {

    private Indexer indexer;

    public ZeroRevolver(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;
    }

    public void initialize() {
        indexer.zeroRevolver();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}