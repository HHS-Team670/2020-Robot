package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Run this command before you run IntakeOneBall
 */
public class RotateToIntakePosition extends CommandBase implements MustangCommand {

    private Indexer indexer;

    public RotateToIntakePosition(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;

    }

    // this should not be needed if readyToIntake/prepareToIntake works the way it
    // should
    @Override
    public void initialize() {
        super.initialize();
        indexer.prepareToIntake();
    }

    @Override
    public void execute() {
        indexer.prepareToIntake();
    }

    @Override
    public void end(boolean isInteruppted) {
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // return done;
        return indexer.isReadyToIntake();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}