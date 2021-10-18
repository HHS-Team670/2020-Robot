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
import frc.team670.robot.commands.routines.AutoIndex;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

/**
 * Autonomous routine starting by shooting 3 balls depending on start position (left or center), go to switch, intake 2
 * front of robot starts in line with initiation line
 * for 2021 field
 * @author elisevbp 
 */
public class LeftShoot2BallSide extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Path trajectory;
    
    public LeftShoot2BallSide(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Turret turret, Shooter shooter) {
        
        //TODO: check if there needs to be a center turret? or it is automatically straight forward
        double turretAng = 0;

     
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        
        trajectory = new Left2Line(driveBase);
        turretAng = RobotConstants.leftTurretAng;

        driveBase.resetOdometry(trajectory.getStartingPose());

        addCommands(
            //shoot all balls from baseline
             new StartShooterByDistance(shooter, driveBase),
             new RotateToAngle(turret, turretAng),
             new Shoot(shooter),

            //TODO: see if shooter needs to be stopped while traversing and not shooting
                
            //from initiation line to 2 balls in line under switch
            new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory, driveBase),
                new AutoIndex(intake, conveyor, indexer, 2)     
            )

            
        );
    }

	@Override
    public void initialize() {
        super.initialize();
        // // Front faces away from wall, heading is 180
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
}
