package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Resets the revolver to the 0 position, defined by a magnet
 */
public class ZeroRevolver extends CommandBase implements MustangCommand {

    private double speed;
    private Indexer indexer;

    public ZeroRevolver(Indexer indexer) {
        speed = 0.5; // TODO: find a speed after testing
        addRequirements(indexer);
        this.indexer = indexer;
    }

    @Override
    public void execute() {
        //int direction = indexer.directionToTurn();
        //indexer.setSpeed(speed * direction);
        indexer.setSpeed(speed);
    }

    @Override
    public void end(boolean isInteruppted) {
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return this.indexer.isZeroed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}