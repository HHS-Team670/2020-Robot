package frc.team670.robot.commands.auton.baseline;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.paths.left.LeftThenForward;
import frc.team670.robot.commands.indexer.RunIndexer;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

/**
 * Shoots then moves the robot back
 */
public class ShootThenForward extends SequentialCommandGroup implements MustangCommand {

    private Path trajectory;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private  Vision coprocessor;
    private Shooter shooter;

    /**
     * Initializes this command from the given parameters
     * 
     * @param startPosition the position of the robot at the beginning of the game
     * @param driveBase the drive base
     * @param intake the intake
     * @param conveyor the conveyor
     * @param shooter the shooter
     * @param indexer the indexer
     * @param turret the turret
     * @param coprocessor the coprocessor
    */
    public ShootThenForward(DriveBase driveBase, Intake intake, Conveyor conveyor,
            Shooter shooter, Indexer indexer, Turret turret, Vision coprocessor) {

        this.driveBase = driveBase;
        this.coprocessor = coprocessor;
        trajectory = new LeftThenForward(driveBase);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStartingPose());
        
        addCommands(
                    new RotateToHome(turret),
                // Get shooter up to speed and aim
                    new StartShooterByDistance(shooter, driveBase), 
                    new Shoot(shooter), 
                    new RunIndexer(indexer),
                    new ParallelCommandGroup(  
                        new StopShooter(shooter),
                        getTrajectoryFollowerCommand(trajectory, driveBase)
        ));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}