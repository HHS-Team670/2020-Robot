package frc.team670.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.subsystems.Indexer;

/**
 * Resets the revolver to the 0 position, defined by a magnet
 */
public class ZeroRevolver extends CommandBase {

    private double speed;
    private Indexer indexer;

    public ZeroRevolver() {
        speed = 0.7;
        indexer = RobotContainer.indexer;
    }

    @Override
    public void execute() {
        int direction = indexer.directionToTurn();
        indexer.setSpeed(speed*direction);
    }

    @Override
    public void end(boolean isInteruppted) {
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.indexer.isZeroed();
    }


}