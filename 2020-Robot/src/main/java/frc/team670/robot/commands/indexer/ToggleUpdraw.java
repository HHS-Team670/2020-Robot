package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class ToggleUpdraw extends InstantCommand implements MustangCommand {

    private Indexer indexer;
    
    public ToggleUpdraw(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        super.initialize();
        indexer.toggleUpdraw();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
    
}