package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Obsolete method
 */
public class Spin extends MustangCommandBase {
    
    private double speed;
    private Indexer indexer;

    public Spin(){
        speed = 0.7;
        addRequirements(RobotContainer.indexer);
        indexer = RobotContainer.indexer;
    }

    public Spin(double speed){
        this.speed = speed;
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

    // not sure what the end condition for continuously spinning is
    @Override
    public boolean isFinished() {
        // one possibility for ending is that all balls are gone
        if (indexer.isEmpty())
            return true;
    
        return false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}