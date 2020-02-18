package frc.team670.robot.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.center.CenterToGenerator2BallSidePath;
import frc.team670.paths.left.LeftToGenerator2BallSidePath;
import frc.team670.paths.right.RightToGenerator2BallSidePath;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.turret.RotateTurretWithVision;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

/**
 * Autonomous routine starting with shooting from the initiation line (facing
 * towards your driver station), ending at (and hopefully intaking and indexing)
 * 2 Power Cells under the generator near your trench.
 * 
 * @author ctychen, meganchoy
 */
public class ShootFromBaseLineThenToGenerator2BallSide extends SequentialCommandGroup implements MustangCommand {

        private Trajectory trajectory;
        private Map<MustangSubsystemBase, HealthState> healthReqs;

        private PIDController leftPIDController = new PIDController(RobotConstants.kPDriveVel,
                        RobotConstants.kIDriveVel, RobotConstants.kDDriveVel);
        private PIDController rightPIDController = new PIDController(RobotConstants.kPDriveVel,
                        RobotConstants.kIDriveVel, RobotConstants.kDDriveVel);

        private enum StartPosition{
                LEFT, CENTER, RIGHT;
        }

        public ShootFromBaseLineThenToGenerator2BallSide(StartPosition startPosition, DriveBase driveBase, Intake intake, 
        Conveyor conveyor, Shooter shooter, Indexer indexer, Turret turret, MustangCoprocessor coprocessor) {
                if (startPosition == StartPosition.LEFT)
                        trajectory = LeftToGenerator2BallSidePath.generateTrajectory(driveBase);
                if (startPosition == StartPosition.CENTER)
                        trajectory = CenterToGenerator2BallSidePath.generateTrajectory(driveBase);
                if (startPosition == StartPosition.RIGHT)
                        trajectory = RightToGenerator2BallSidePath.generateTrajectory(driveBase);
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
                                new RamseteCommand(trajectory, driveBase::getPose,
                                        new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
                                        new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
                                        RobotConstants.kaVoltSecondsSquaredPerMeter),
                                        RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController, rightPIDController,
                                        // RamseteCommand passes volts to the callback
                                        driveBase::tankDriveVoltage, driveBase),
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