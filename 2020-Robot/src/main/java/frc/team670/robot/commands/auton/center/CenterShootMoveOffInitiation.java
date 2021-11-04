package frc.team670.robot.commands.auton.center;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.paths.center.Center3Line;
import frc.team670.robot.commands.indexer.RunIndexer;
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

/*
 * Autonomous routine starting by shooting 3 balls from center then goes forwards towards the port and off initiation line
 */
public class CenterShootMoveOffInitiation extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Path trajectory;

    public CenterShootMoveOffInitiation(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret,
            Shooter shooter, Vision coprocessor) {

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        trajectory = new Center3Line(driveBase);

        driveBase.resetOdometry(trajectory.getStartingPose());

        addCommands(

                // 1) shoot 3 balls from initiation line
                new ZeroTurret(turret),
                new ParallelCommandGroup(
                    new StartShooter(shooter), // flywheel starts turning
                    new RotateToAngle(turret, 0)
                ),
                new RunIndexer(indexer, conveyor),
                getTrajectoryFollowerCommand(trajectory, driveBase)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        // // Front faces away from wall, heading is 180
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}