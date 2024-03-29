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
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.commands.shooter.StartShooter;
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
 * back of robot on initiation line (closer to trenc)
 * 
 * @author Elise V, justin h, rishabh b
 */
public class RightShootTrench extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Path trajectory;

    public RightShootTrench(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret,
            Shooter shooter, Vision coprocessor) {

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);

        trajectory = new RightThroughTrench(driveBase);

        driveBase.resetOdometry(trajectory.getStartingPose());

        addCommands(
                // shoot 3 balls from initiation line
                new ParallelCommandGroup(new ZeroTurret(turret), new StartShooter(shooter) // flywheel starts turning
                ), new RotateToAngle(turret, -30), new RunIndexer(indexer, conveyor), new DeployIntake(true, intake),
                new ParallelCommandGroup(getTrajectoryFollowerCommand(trajectory, driveBase),
                        new AutoIndex(intake, conveyor, indexer, 3), new RotateToAngle(turret, -12.75),
                        new SetRPMTarget(2850, shooter), new StartShooter(shooter)),
                new RunIndexer(indexer, conveyor));
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}