package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.drive.straight.TimedDrive;
import frc.team670.robot.commands.indexer.EmptyRevolver;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class ToTrenchRunAndShoot extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public ToTrenchRunAndShoot(double initAng, DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret, Shooter shooter) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);

        addCommands(
             new RotateToHome(turret),
            //     new ParallelCommandGroup(
            //         new StartShooter(shooter), 
            //         new RotateToAngle(turret, initAng)
            //     ),
            //     new Shoot(shooter), 
            //         // new StageOneBallToShoot(indexer),
            //     new EmptyRevolver(indexer),
            //     new TimedDrive(1, speed, speed, driveBase),
                new DeployIntake(true, intake),
                new TimedDrive(1.5, 0.3, 0.3, driveBase),
                new ParallelCommandGroup(
                    new TimedDrive(3, 0.12, 0.12, driveBase),
                    new RotateToAngle(turret, initAng),
                    new IntakeBallToIndexer(intake, conveyor, indexer).withTimeout(3.7)
                ),
                new StartShooter(shooter),
                new Shoot(shooter),
                new EmptyRevolver(indexer),
                new StopShooter(shooter)
        );

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}