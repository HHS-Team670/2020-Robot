package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class UnjamIndexer extends CommandBase implements MustangCommand {

    private Indexer indexer;

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public UnjamIndexer(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute(){
        // "Wiggle" the indexer back and forth a bit to unjam?
    }

    @Override
    public boolean isFinished(){
        // true when indexer rotator current is back to normal (unjammed)
        return false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}