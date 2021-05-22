package frc.team670.robot.commands.auton.twentytwentyone;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.drive.straight.TimedDrive;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.paths.twentytwentyone.RightShoot3Get3Shoot3Get3_GODSPEED;
import frc.team670.robot.commands.indexer.EmptyRevolver;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

public class Shoot3Get3Shoot3Get3_GODSPEED extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Path trajectory;
    
    public Shoot3Get3Shoot3Get3_GODSPEED(DriveBase driveBase, Intake intake, Conveyor conveyor, 
    Indexer indexer, Turret turret, Shooter shooter) {
        
        this.driveBase = driveBase;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        this.trajectory = new RightShoot3Get3Shoot3Get3_GODSPEED(driveBase);

        driveBase.resetOdometry(trajectory.getStartingPose());

        addCommands(
                getTrajectoryFollowerCommand(trajectory, driveBase)
        );

    }

    @Override
    public void initialize() {
        super.initialize();
        // Front faces away from wall, heading is 180
        driveBase.resetOdometry(new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(0)));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
}