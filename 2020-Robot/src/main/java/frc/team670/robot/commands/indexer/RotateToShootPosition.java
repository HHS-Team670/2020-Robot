package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Rotates a certain chamber in the indexer (0, 1, 2, 3, or 4) to the top
 */
public class RotateToShootPosition extends MustangCommandBase {

    private double speed;
    private Indexer indexer;

    public RotateToShootPosition(){
        speed = 0.8;
        addRequirements(RobotContainer.indexer);
        indexer = RobotContainer.indexer;
    }

    @Override
    public void execute() {
        indexer.setSpeed(speed);

    }

    @Override
    public void end(boolean isInteruppted) {
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
    
        return indexer.getCurrentChamber() == indexer.getFirstFull();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}