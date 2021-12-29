package frc.team670.robot.commands.auton.left;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.paths.left.Left2Line;
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

/**
 * Autonomous routine starting by shooting 3 balls from the left then goes forward towards the port and off initiation line
 * front of robot starts in line with initiation line for 2021 field
 * @author elisevbp 
 */
public class LeftShootMoveOffInitiation extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Path trajectory;
    
    public LeftShootMoveOffInitiation(DriveBase driveBase, Intake intake, 
        Conveyor conveyor, Indexer indexer, Turret turret, Shooter shooter) {

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        
        trajectory = new Left2Line(driveBase);

        driveBase.resetOdometry(trajectory.getStartingPose());

        addCommands(
            //shoot all balls from baseline
            new ZeroTurret(turret),
            new ParallelCommandGroup(
                new StartShooter(shooter), // flywheel starts turning
                new RotateToAngle(turret, 0) 
            ),
            new RunIndexer(indexer, conveyor), // indexer runs 
            //from initiation line forwards toward port
            getTrajectoryFollowerCommand(trajectory, driveBase)
        );
    }

	@Override
    public void initialize() {
        super.initialize();
        // Front faces away from wall, heading is 180
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
