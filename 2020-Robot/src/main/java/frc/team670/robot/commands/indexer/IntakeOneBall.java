package frc.team670.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.subsystems.Indexer;

/**
 * Detects when the ball is fully in, then rotates one compartment over
 */
public class IntakeOneBall extends CommandBase {

    private boolean ballIn;
    private double speed;
    private double rotationGoal;
    private Indexer indexer;

    public IntakeOneBall() {
        ballIn = false;
        speed = 0.7;
        rotationGoal = 0;
        indexer = RobotContainer.indexer;
    }



    @Override
    public void execute() {
        if (RobotContainer.indexer.ballIn()) {
            ballIn = true;
            indexer.fillChamber(indexer.getCurrentChamber() + 3);// might be + 2 
        }

        if (ballIn && indexer.getSpeed() == 0) {
            rotationGoal = indexer.getPosition() + 0.2;
            RobotContainer.indexer.setSpeed(speed);
        }

    }

    @Override
    public boolean isFinished() {

        //Makes sure it doesn't try to take in more balls when it's full
        if (indexer.totalNumOfBalls() == 5) {
            return true;
        }

        if (indexer.getSpeed() < 0) {
            return indexer.getPosition() < rotationGoal;
        } else {
            return indexer.getPosition() > rotationGoal;
        }
    }

    @Override
    public void end(boolean isInteruppted) {
        indexer.setNumBalls(indexer.totalNumOfBalls() + 1);
        indexer.setSpeed(0);
    }



    
}