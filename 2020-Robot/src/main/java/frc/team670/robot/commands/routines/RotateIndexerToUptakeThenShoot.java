package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.StageOneBallToShoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class RotateIndexerToUptakeThenShoot extends SequentialCommandGroup implements MustangCommand {

    private Indexer indexer;
    private Shooter shooter;
    // private Turret turret;

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public RotateIndexerToUptakeThenShoot(Indexer indexer, Shooter shooter) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer, shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        
        // Stage and uptake 1 ball while preparing the shooter
        addCommands(
            new ParallelCommandGroup(
            new StageOneBallToShoot(indexer),
            new StartShooter(shooter))
            );

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}