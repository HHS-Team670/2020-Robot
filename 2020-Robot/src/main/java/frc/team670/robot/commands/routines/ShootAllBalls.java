package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.indexer.RunIndexer;
import frc.team670.robot.commands.intake.RunConveyor;
import frc.team670.robot.commands.shooter.StartShooterByPoseDistance;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Shooter;

public class ShootAllBalls extends SequentialCommandGroup implements MustangCommand {

    private Indexer indexer;
    private Conveyor conveyor;
    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public ShootAllBalls(Indexer indexer, Conveyor conveyor, Shooter shooter, DriveBase driveBase){
        this.indexer = indexer;
        this.shooter = shooter;
        this.conveyor = conveyor;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        addCommands(
            new StartShooterByPoseDistance(shooter, driveBase),
            new ParallelCommandGroup(
                new RunIndexer(indexer, conveyor),
                new RunConveyor(false, conveyor, indexer))
            );
    }

    /**
     * This sequence will only end if we tell it to, otherwise, we keep blasting
     */
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stopUpdraw();
        conveyor.stop();

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }
    
}