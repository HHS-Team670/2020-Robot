package frc.team670.robot.commands.auton.right;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.paths.right.RightThroughTrench;
import frc.team670.robot.commands.indexer.RunIndexer;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.routines.AutoIndex;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.commands.turret.ZeroTurret;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

/**
 * Trench Shoot routine for Chezy 2021 (workshop 10/12/2021) google doc link:
 * https://docs.google.com/document/d/1GCqiZlTvnIp7UbRZ-_Gu2sK9tljfNCpqdYkApQ3Qdtk/edit?usp=sharing
 * back of robot on initiation line
 * @author Elise V, justin h, rishabh b
 */
public class RightShootTrench extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Path trajectory1;

    public RightShootTrench(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret,
            Shooter shooter, Vision coprocessor) {

        // double turretAng = 0;

        this.driveBase = driveBase;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        trajectory1 = new RightThroughTrench(driveBase);
        // turretAng = RobotConstants.rightTurretAng;

        driveBase.resetOdometry(trajectory1.getStartingPose());

        addCommands(
                // 1) shoot 3 balls from initiation line
                new ZeroTurret(turret),
                new StartShooterByDistance(shooter, driveBase), // flywheel starts turning
                new RotateToAngle(turret, -25), //
                new RunIndexer(indexer), // indexer runs lol
                new DeployIntake(true, intake), 
                new ParallelCommandGroup(
                        getTrajectoryFollowerCommand(trajectory1, driveBase), 
                        new AutoIndex(intake, conveyor, indexer, 3)),
                new StartShooterByDistance(shooter, driveBase), // flywheel starts turning
                new RotateToAngle(turret, -10.75), //
                new RunIndexer(indexer) // indexer runs lol
                

        );
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