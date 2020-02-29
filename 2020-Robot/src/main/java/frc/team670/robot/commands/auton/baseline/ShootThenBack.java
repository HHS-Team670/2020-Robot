package frc.team670.robot.commands.auton.baseline;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.Path;
import frc.team670.paths.center.CenterToTrenchPath;
import frc.team670.paths.center.CenterThenBack;
import frc.team670.paths.left.LeftToTrenchPath;
import frc.team670.paths.right.RightToTrenchPath;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.auton.UpdatePoseFromVision;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.commands.turret.RotateTurret;
import frc.team670.robot.commands.vision.GetVisionData;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

public class ShootThenBack extends SequentialCommandGroup implements MustangCommand {

    private Path trajectory;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private MustangCoprocessor coprocessor;
    private Shooter shooter;

    public ShootThenBack(DriveBase driveBase, Intake intake, Conveyor conveyor,
            Shooter shooter, Indexer indexer, Turret turret, MustangCoprocessor coprocessor) {

        this.driveBase = driveBase;
        this.coprocessor = coprocessor;
        trajectory = new CenterThenBack(driveBase);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);

        addCommands(
                //     new RotateToHome(turret),
                // // Get shooter up to speed and aim
                //     new StartShooter(shooter), 
                //     new Shoot(shooter), 
                //     new SendAllBalls(indexer),
                //     new ParallelCommandGroup(  
                //         new StopShooter(shooter),
                        getTrajectoryFollowerCommand(trajectory, driveBase)
                    );
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}