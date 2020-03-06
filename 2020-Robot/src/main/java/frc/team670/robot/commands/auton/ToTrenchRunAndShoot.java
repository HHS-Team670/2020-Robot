package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.drive.straight.TimedDrive;
import frc.team670.robot.commands.indexer.EmptyRevolver;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Shoot from baseline then drive to trench run, hopefully intake and shoot 3 balls.
 * So far you will only do this if you start on line to the right of the target.
 */
public class ToTrenchRunAndShoot extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private static final double FROM_TRENCH_TURRET_ANGLE = -12.001;

    /**
     * @param initAng angle (degrees) the turret should turn to for shooting from baseline at beginning
     * @param trenchAng angle (degrees) the turret should be turned to when ready to shoot from the trench
     */
    public ToTrenchRunAndShoot(double initAng, DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret, Shooter shooter) {
        
        this.driveBase = driveBase;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);

        addCommands(
                // Shoots balls from baseline
                new RotateToHome(turret),
                new ParallelCommandGroup(
                    new StartShooter(shooter), 
                    new RotateToAngle(turret, initAng) // turret angle for shooting when starting on baseline to right
                ),
                new Shoot(shooter), 
                new EmptyRevolver(indexer),

                // Going to trench to pick up balls, shooter can still be running
                new DeployIntake(true, intake),
                new ParallelCommandGroup(     
                    new TimedDrive(1, 0.3, driveBase),
                    new SetRPMTarget(2850, shooter)
                ),
                new ParallelCommandGroup(
                    new IntakeBallToIndexer(intake, conveyor, indexer).withTimeout(5.2),
                    new TimedDrive(4.2, 0.12, driveBase),
                    new RotateToAngle(turret, FROM_TRENCH_TURRET_ANGLE)
                ),
                new StartShooterByDistance(shooter, driveBase),
                new Shoot(shooter),
                new EmptyRevolver(indexer),
                new StopShooter(shooter)
        );

    }

    @Override
    public void initialize() {
        super.initialize();
        // Front faces away from wall, heading is 180
        driveBase.resetOdometry(new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(180)));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }

}