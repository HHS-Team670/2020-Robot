package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.drive.straight.TimedDrive;
import frc.team670.robot.commands.indexer.EmptyRevolver;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.indexer.StageOneBallToShoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.commands.turret.ZeroTurret;
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

public class ShootFromAngleThenTimeDrive extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Pose2d startPose;

    /**
     * 
     * @param turretAng Angle the turret should turn to at the beginning for shooting
     * @param waitTime The delay (s) between shooting and driving, if no delay use 0
     * @param speed Drivebase percent output. Negative reverse, positive forward
     */
    public ShootFromAngleThenTimeDrive(Pose2d startPose, double turretAng, double waitTime, double speed, DriveBase driveBase, Intake intake, Conveyor conveyor,
            Shooter shooter, Indexer indexer, Turret turret) {
        this.driveBase = driveBase;
        this.startPose = startPose;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);

        addCommands(
                // Get shooter up to speed and aim
                new ParallelCommandGroup(
                    new StartShooter(shooter), 
                    new RotateToAngle(turret, turretAng)
                ),
                new Shoot(shooter), 
                    // new StageOneBallToShoot(indexer),
                new EmptyRevolver(indexer),

                new WaitCommand(waitTime), // Delay moving after shot if needed

                new ParallelCommandGroup(
                    new TimedDrive(1, speed, driveBase),
                    new StopShooter(shooter)
                )
            );
    }

    @Override
    public void initialize() {
        driveBase.resetOdometry(this.startPose);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}