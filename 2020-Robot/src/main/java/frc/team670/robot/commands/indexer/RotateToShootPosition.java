package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Run this commmand before running SendAllBalls
 */
public class RotateToShootPosition extends CommandBase implements MustangCommand {

    private Indexer indexer;

    public RotateToShootPosition(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;

    }

    public void initialize() {
        super.initialize();
        indexer.prepareToShoot();
    }

    @Override
    public void execute() {
        indexer.prepareToShoot();
    }

    @Override
    public void end(boolean isInteruppted) {
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // return done;
        return indexer.isReadyToShoot();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}