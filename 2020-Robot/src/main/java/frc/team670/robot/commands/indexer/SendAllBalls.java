package frc.team670.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.subsystems.Indexer;

/**
 * Sends al the balls to the shooter
 */
public class SendAllBalls extends CommandBase {

    private double speed;
    private Indexer indexer;
    private int originalNumBalls, ballsLeft;
    private double[] rotationOrder;
    private double originalPosition;
    private int rotationDirection;
    private boolean reachedBall;

    public SendAllBalls() {
        speed = 0.7;
        indexer = RobotContainer.indexer;
        originalPosition = indexer.getPosition();
        originalNumBalls = indexer.totalNumOfBalls();
        ballsLeft = originalNumBalls;
        rotationDirection = 0;
        reachedBall = false;
        if (originalNumBalls == 1) {
            rotationOrder = new double[]{0.2+0.1};
        } else if (originalNumBalls == 2) {
            rotationOrder = new double[]{0.1, 0.1+0.2};
        } else if (originalNumBalls == 3) {
            rotationOrder = new double[]{-0.1, 0.1, 0.1+0.2};
        } else if (originalNumBalls == 4) {
            rotationOrder = new double[]{0.1, 0.1+0.2, 0.1+0.2*3, 0.1+0.2*4};   
        } else if (originalNumBalls == 5) {
            rotationOrder = new double[]{0.1, 0.1+0.2, 0.1+0.2*2, 0.1+0.2*3, 0.1+0.2*3};   
        } else {
            //Nice, either it's 0 or something is going amazingly
        }
    }



    @Override
    public void execute() {
        
        //Make sure to have whatever subsystem handles moving the balls out update the number of balls on the indexer

        if (!reachedBall) {
            indexer.setSpeed(speed*rotationDirection);
            if (ballsLeft == originalNumBalls) {
                if (rotationOrder[0] > 0) {
                    rotationDirection = 1;
                } else {
                    rotationDirection = -1;
                }
            } else {
                if (rotationOrder[originalNumBalls-ballsLeft] > rotationOrder[originalNumBalls-ballsLeft-1]) {
                    rotationDirection = 1;
                } else {
                    rotationDirection = -1;
                }
            }
    
            if (rotationDirection == 1) {
                if (indexer.getPosition() > originalPosition + rotationOrder[originalNumBalls-ballsLeft]) {
                    reachedBall = true;
                } else {
                    reachedBall = false;
                }
            } else if (rotationDirection == -1) {
                if (indexer.getPosition() < originalPosition +  rotationOrder[originalNumBalls-ballsLeft]) {
                    reachedBall = true;
                } else {
                    reachedBall = false;
                }
            }
        }
        


    }

    @Override
    public boolean isFinished() {
        return indexer.totalNumOfBalls() == 0;
    }

    @Override
    public void end(boolean isInteruppted) {
        CommandScheduler.getInstance().schedule(new ZeroRevolver());
    }



    
}