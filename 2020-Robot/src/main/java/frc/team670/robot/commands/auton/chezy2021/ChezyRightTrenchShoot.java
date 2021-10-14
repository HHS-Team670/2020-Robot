package frc.team670.robot.commands.auton.chezy2021;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.paths.Path;
import frc.team670.paths.chezy2021.CenterThroughTrench;
import frc.team670.paths.chezy2021.RightThroughTrench_GODSPEED2021Pt1;
import frc.team670.paths.chezy2021.RightThroughTrench_GODSPEED2021Pt2;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.indexer.ToggleUpdraw;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.AutoIndex;
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.shooter.StopUpdraw;
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
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.commands.indexer.RunIndexer;

/**
 * Trench Shoot routine for Chezy 2021 (workshop 10/12/2021) google doc link:
 * https://docs.google.com/document/d/1GCqiZlTvnIp7UbRZ-_Gu2sK9tljfNCpqdYkApQ3Qdtk/edit?usp=sharing
 * back of robot on initiation line
 * @author Elise V justin h rishabh b
 */
public class ChezyRightTrenchShoot extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Path trajectory1, trajectory2;

    public ChezyRightTrenchShoot(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret,
            Shooter shooter, Vision coprocessor) {

        double turretAng = 0;
        // double trenchTurretAng = RobotConstants.trenchTurretAng;

        this.driveBase = driveBase;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        // if (startPosition == StartPosition.RIGHT) {
        trajectory1 = new RightThroughTrench_GODSPEED2021Pt1(driveBase);
        trajectory2 = new RightThroughTrench_GODSPEED2021Pt2(driveBase);
        turretAng = RobotConstants.rightTurretAng;
        // }
        // if (startPosition == StartPosition.CENTER)
        // trajectory1 = new CenterThroughTrench(driveBase);

        // TODO: reset to trajectory 1
        driveBase.resetOdometry(trajectory1.getStartingPose());

        // if (startPosition == StartPosition.RIGHT) {
        addCommands(
                // 1) shoot 3 balls from initiation line
                new StartShooterByDistance(shooter, driveBase), // flywheel starts turning
                new RotateTurret(turret, driveBase, coprocessor), //
                new RunIndexer(indexer), // indexer runs lol
                new Shoot(shooter),
                // TODO: see if shooter needs to be stopped while traversing and not shooting

                // 2) goes through the trench and intakes
                new DeployIntake(true, intake), 
                new ParallelCommandGroup(
                        getTrajectoryFollowerCommand(trajectory1, driveBase), 
                        new AutoIndex(intake, conveyor, indexer, 3))

        // //shoot all balls from baseline
        // new StartShooterByDistance(shooter, driveBase),
        // new RotateToAngle(turret, turretAng),
        // new Shoot(shooter),
        // new SendAllBalls(indexer),
        // new RotateToNextChamber(indexer, true),
        // // //TODO: see if shooter needs to be stopped while traversing and not
        // shooting

        // //from initiation line to start of trench
        // new ParallelCommandGroup(
        // getTrajectoryFollowerCommand(trajectory1, driveBase),
        // new IntakeBallToIndexer(intake, conveyor, indexer).withTimeout(2)
        // ),
        // new StopUpdraw(indexer),

        // // from start of trench to end of trench w 3 balls intake
        // new ParallelCommandGroup(
        // // new RotateToIntakePosition(indexer),
        // getTrajectoryFollowerCommand(trajectory2, driveBase)
        // ),

        // //shoot from color wheel,
        // //TODO: find new turretAng

        // //new StartShooterByDistance(shooter, driveBase),
        // new SetRPMTarget(2750, shooter),
        // new StartShooter(shooter),
        // new RotateToAngle(turret, trenchTurretAng),
        // new Shoot(shooter),
        // new Send3BallsWait(indexer)
        // //new EmptyRevolver(indexer)

        );
        // }

        // if (startPosition == StartPosition.CENTER) {
        // addCommands(
        // //shoot all balls from baseline
        // // new StartShooterByDistance(shooter, driveBase),
        // // new RotateToAngle(turret, turretAng),
        // // new Shoot(shooter),
        // // new EmptyRevolver(indexer),

        // //TODO: see if shooter needs to be stopped while traversing and not shooting

        // //from initiation line to end of trench w 3 balls intaked
        // new ParallelCommandGroup(
        // getTrajectoryFollowerCommand(trajectory1, driveBase),
        // new IntakeBallToIndexer(intake, conveyor, indexer).withTimeout(3.2)
        // ),

        // //shoot from color wheel,
        // //TODO: find new turretAng
        // // turretAng = ...;
        // // new StartShooterByDistance(shooter, driveBase),
        // // new RotateToAngle(turret, turretAng),
        // // new Shoot(shooter),
        // //new EmptyRevolver(indexer),

        // getTrajectoryFollowerCommand(trajectory3, driveBase),

        // new ParallelCommandGroup(
        // getTrajectoryFollowerCommand(trajectory4, driveBase),
        // new IntakeBallToIndexer(intake, conveyor, indexer)
        // )

        // );
        // }

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
}