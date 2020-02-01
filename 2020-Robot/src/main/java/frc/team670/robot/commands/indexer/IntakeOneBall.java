package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Detects when the ball is fully in, then rotates one compartment over Make
 * sure RotateToIntakePosition has been run
 */
public class IntakeOneBall extends CommandBase implements MustangCommand {

    private boolean ballIn;
    private double speed;
    private double rotationGoal;
    private Indexer indexer;

    public IntakeOneBall(Indexer indexer) {
        ballIn = false;
        speed = 0.7;
        rotationGoal = 0;
        addRequirements(indexer);
        this.indexer = indexer;
    }

    @Override
    public void execute() {
        if (this.indexer.ballIn()) {
            ballIn = true;
            indexer.fillChamber(indexer.getBottomChamber());
        }

        if (ballIn && indexer.getSpeed() == 0) {
            rotationGoal = indexer.getPosition() + 0.2;
            this.indexer.setSpeed(speed);
        }

    }

    @Override
    public boolean isFinished() {

        // Makes sure it doesn't try to take in more balls when it's full
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
        indexer.setSpeed(0);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}