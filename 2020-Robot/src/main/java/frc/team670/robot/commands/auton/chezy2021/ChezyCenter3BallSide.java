package frc.team670.robot.commands.auton.chezy2021;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
// import frc.team670.paths.left.LeftStraightThenBack;
// import frc.team670.paths.twentytwentyone.Center3Line;
// import frc.team670.paths.twentytwentyone.Left3Line;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
// import frc.team670.robot.commands.indexer.EmptyRevolver;
// import frc.team670.robot.commands.indexer.RotateToIntakePosition;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.AutoIndex;
// import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.commands.turret.RotateTurret;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.paths.chezy2021.Center3Line;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;
import frc.team670.mustanglib.commands.drive.straight.TimedDrive;
import frc.team670.robot.commands.indexer.*;
import frc.team670.robot.commands.shooter.*;

/**
 * Autonomous routine starting by shooting 3 balls from center, go to switch,
 * intake 3 front of robot starts in line with initiation line for 2021 field
 * 
 * @author elisevbp, justin h
 */
public class ChezyCenter3BallSide extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    // private DriveBase driveBase;
    private Path trajectory;

    public ChezyCenter3BallSide(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret,
            Shooter shooter, Vision coprocessor) {

        // TODO: check if there needs to be a center turret? or it is automatically
        // straight forward
        // double turretAng = RobotConstants.;
        double turretAng = 0;

        // this.driveBase = driveBase;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        // if (startPosition == StartPosition.LEFT) {
        // trajectory = new Center3Line(driveBase);
        // //trajectory = new LeftStraightThenBack(driveBase);
        // turretAng = RobotConstants.leftTurretAng;
        // }
        // if (startPosition == StartPosition.CENTER) {
        trajectory = new Center3Line(driveBase);
        turretAng = -12.5; // facing straight forward
        // }

        driveBase.resetOdometry(trajectory.getStartingPose());

        addCommands(

                // 1) shoot 3 balls from initiation line
                new StartShooterByDistance(shooter, driveBase), // flywheel starts turning
                new RotateTurret(turret, driveBase, coprocessor), //
                new RunIndexer(indexer), // indexer runs lol
                new Shoot(shooter),
                // shoot all balls from baseline
                // new StartShooterByDistance(shooter, driveBase),
                // new SetRPMTarget(2600, shooter),
                // new StartShooter(shooter),
                // new RotateToAngle(turret, turretAng), //TODO on left, doesn't rotate all the
                // way to leftTurretAng

                // 2) goes under switch and intakes 3 balls under switch
                new DeployIntake(true, intake),
                new ParallelCommandGroup(
                    getTrajectoryFollowerCommand(trajectory, driveBase),
                    new AutoIndex(intake, conveyor, indexer)
                )
        );

                // TODO: see if shooter needs to be stopped while traversing and not shooting
    }

    @Override
    public void initialize() {
        super.initialize();
        // // Front faces away from wall, heading is 180
        // driveBase.resetOdometry(new
        // Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS,
        // FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(0)));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }

}