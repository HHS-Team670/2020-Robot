package frc.team670.robot.commands.auton.baseline;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.Path;
import frc.team670.paths.center.CenterToGenerator2BallSidePath;
import frc.team670.paths.left.LeftToGenerator2BallSidePath;
import frc.team670.paths.right.RightToGenerator2BallSidePath;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.turret.RotateTurretWithVision;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;

/**
 * Autonomous routine starting with shooting from the initiation line (facing
 * towards your driver station), ending at (and hopefully intaking and indexing)
 * 2 Power Cells under the generator near your trench.
 * 
 * @author ctychen, meganchoy
 */
public class ShootFromBaseLineThenToGenerator2BallSide extends SequentialCommandGroup implements MustangCommand {

        private Path trajectory;
        private Map<MustangSubsystemBase, HealthState> healthReqs;

        public ShootFromBaseLineThenToGenerator2BallSide(StartPosition startPosition, DriveBase driveBase, Intake intake, 
        Conveyor conveyor, Shooter shooter, Indexer indexer, Turret turret, MustangCoprocessor coprocessor) {
                if (startPosition == StartPosition.LEFT)
                        trajectory = new LeftToGenerator2BallSidePath(driveBase);
                if (startPosition == StartPosition.CENTER)
                        trajectory = new CenterToGenerator2BallSidePath(driveBase);
                if (startPosition == StartPosition.RIGHT)
                        trajectory = new RightToGenerator2BallSidePath(driveBase);
                healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
                healthReqs.put(driveBase, HealthState.GREEN);
                healthReqs.put(shooter, HealthState.GREEN);
                healthReqs.put(intake, HealthState.GREEN);
                healthReqs.put(conveyor, HealthState.GREEN);
                healthReqs.put(indexer, HealthState.GREEN);
                healthReqs.put(turret, HealthState.GREEN);
                addCommands(
                        new ParallelCommandGroup(
                                new StartShooter(shooter),
                                new RotateTurretWithVision(turret, coprocessor)
                        ),
                         
                        new ParallelCommandGroup(
                                new Shoot(shooter), 
                                new SendAllBalls(indexer)
                        ),

                        new ParallelCommandGroup (
                                getTrajectoryFollowerCommand(trajectory, driveBase),
                                new IntakeBallToIndexer(intake, conveyor, indexer)       
                        )
                );

        }

        @Override
        public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
                // maybe also check that NavX is there
                return healthReqs;
        }
}