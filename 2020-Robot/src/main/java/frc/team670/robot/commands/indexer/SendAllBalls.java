package frc.team670.robot.commands.indexer;

import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Sends all the balls to the shooter, use when all 5 chambers are filled to rapidly dispense balls
 */
public class SendAllBalls extends SequentialCommandGroup implements MustangCommand {

    private Indexer indexer;

    public SendAllBalls(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;
        addCommands(new SendOneBall(indexer), new SendOneBall(indexer), new SendOneBall(indexer),
            new SendOneBall(indexer), new SendOneBall(indexer));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        indexer.uptake(0.0);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        return healthReqs;
    }



    
}