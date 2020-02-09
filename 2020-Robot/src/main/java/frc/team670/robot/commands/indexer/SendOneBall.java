package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Sends one ball to the shooter
 */
public class SendOneBall extends WaitCommand implements MustangCommand {

    private Indexer indexer;

    public SendOneBall(Indexer indexer) {
        super(10); //TODO: test how many seconds uptake should spin
        addRequirements(indexer);
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        if (!indexer.updrawIsUpToSpeed())
            indexer.rotateToLoadShoot();
    }

    @Override
    public void execute() {
        indexer.uptake(0.5); // Find speed after testing
    }
    
    @Override
    public void end(boolean isInteruppted) {
        indexer.prepareToShoot();
    }

    @Override
    public boolean isFinished() {
        return indexer.updrawIsUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }



    
}