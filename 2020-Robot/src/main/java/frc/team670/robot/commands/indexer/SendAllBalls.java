package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.MustangScheduler;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Sends all the balls to the shooter
 */
public class SendAllBalls extends CommandBase implements MustangCommand {

    private double speed;
    private Indexer indexer;
    private double originalPosition;
    private double rotationGoal;
    private int originalNumBalls;
    private int desiredNumBalls;

    public SendAllBalls() {
        speed = 0.7;
        indexer = RobotContainer.indexer;
        originalNumBalls = indexer.totalNumOfBalls();
        originalPosition = indexer.getPosition();
        desiredNumBalls = indexer.totalNumOfBalls();
        rotationGoal = indexer.getPosition();
    }

    @Override
    public void initialize() {
        indexer.setSpeed(speed);
    }

    @Override
    public void execute() {

        // Make sure to have whatever subsystem handles moving the balls out update the
        // number of balls on the indexer
        if (indexer.getPosition() >= rotationGoal) {
            desiredNumBalls--;

            if (indexer.getSpeed() != 0) {
                indexer.setSpeed(0);
            }

            //SPIN UPTAKE WHEELS HERE

            // Means that the ball left the revolver
            if (indexer.totalNumOfBalls() == desiredNumBalls) {
                rotationGoal = originalPosition + 0.2 + (originalNumBalls - desiredNumBalls) * 0.2;
            }
        }

    }

    @Override
    public boolean isFinished() {
        return indexer.totalNumOfBalls() == 0;
    }

    @Override
    public void end(boolean isInteruppted) {
        
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }



    
}