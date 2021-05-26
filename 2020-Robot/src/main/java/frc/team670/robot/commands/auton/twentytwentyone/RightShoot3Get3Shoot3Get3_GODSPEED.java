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
import frc.team670.mustanglib.utils.Logger;
import frc.team670.paths.Path;
import frc.team670.paths.twentytwentyone.RightThroughTrench_GODSPEED2021;
import frc.team670.paths.twentytwentyone.TrenchTo3BallLine_GODSPEED2021;
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
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

public class RightShoot3Get3Shoot3Get3_GODSPEED extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Path trajectory1, trajectory2;
    
    public RightShoot3Get3Shoot3Get3_GODSPEED(DriveBase driveBase, Intake intake, Conveyor conveyor, 
    Indexer indexer, Turret turret, Shooter shooter) {
        
        double turretAng = RobotConstants.rightTurretAng;

        this.driveBase = driveBase;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        // this.trajectory = new RightShoot3Get3Shoot3Get3_GODSPEED(driveBase);
        this.trajectory1 = new RightThroughTrench_GODSPEED2021(driveBase);
        this.trajectory2 = new TrenchTo3BallLine_GODSPEED2021(driveBase);

        driveBase.resetOdometry(trajectory1.getStartingPose());
        //Logger.consoleLog("running godspeed from right");

        addCommands(

            //shoot all balls from baseline
            // new StartShooterByDistance(shooter, driveBase),
            // new RotateToAngle(turret, turretAng),
            // new Shoot(shooter),
            // new EmptyRevolver(indexer),

            //TODO: see if shooter needs to be stopped while traversing and not shooting

            //from initiation line to end of trench w 3 balls intaked
            new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory1, driveBase),
                new IntakeBallToIndexer(intake, conveyor, indexer)       
            )

            //shoot from color wheel, 
            //TODO: find new turretAng 
            // turretAng = ...;
            // new StartShooterByDistance(shooter, driveBase),
            // new RotateToAngle(turret, turretAng),
            // new Shoot(shooter),
            // new EmptyRevolver(indexer),

            // new ParallelCommandGroup(
            //     getTrajectoryFollowerCommand(trajectory2, driveBase),
            //     new IntakeBallToIndexer(intake, conveyor, indexer)       
            // )
        );

    }

    @Override
    public void initialize() {
        super.initialize();
        // // Front faces away from wall, heading is 180
        // driveBase.resetOdometry(new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(0)));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
}