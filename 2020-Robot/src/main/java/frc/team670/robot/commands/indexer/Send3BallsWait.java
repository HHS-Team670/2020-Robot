package frc.team670.robot.commands.indexer;

import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;

/**
 * Sends all balls to the shooter. Sends one, waits, sends another, waits, and then sends 3 fast (for auton)
 */
public class Send3BallsWait extends SequentialCommandGroup implements MustangCommand {

    private Indexer indexer;
    private double waitSeconds = 0.3;

    public Send3BallsWait(Indexer indexer) {
        addRequirements(indexer);
        this.indexer = indexer;
        addCommands(new StageOneBallToShoot(indexer), new SendOneBallToShoot(indexer), new WaitCommand(waitSeconds), new StageOneBallToShoot(indexer),
                new SendOneBallToShoot(indexer), new WaitCommand(waitSeconds), new StageOneBallToShoot(indexer), new SendOneBallToShoot(indexer),
                new StageOneBallToShoot(indexer), new SendOneBallToShoot(indexer), new StageOneBallToShoot(indexer),
                new SendOneBallToShoot(indexer));
    }

    @Override
    public void end(boolean interrupted) {
        indexer.updraw(false);
        Logger.consoleLog("SendAllBalls Ended");
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        return healthReqs;
    }

}