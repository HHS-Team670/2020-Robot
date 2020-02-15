package frc.team670.robot.auton.centerDS;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.RotateToIntakePosition;
import frc.team670.robot.commands.intake.RunConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Autonomous routine starting with shooting from the middle of the
 * initiation line (facing towards your driver station), ending at
 * (and hopefully intaking and indexing)
 * 2 Power Cells under the generator near your trench.
 */
public class CenterToGenerator2BallSide extends SequentialCommandGroup implements MustangCommand {

        private DriveBase driveBase;
        private Shooter shooter;
        private Intake intake;
        private Conveyor conveyor;
        private Indexer indexer;
        private Trajectory trajectory;
        private Map<MustangSubsystemBase, HealthState> healthReqs;

        private PIDController leftPIDController = new PIDController(RobotConstants.kPDriveVel,
                        RobotConstants.kIDriveVel, RobotConstants.kDDriveVel);
        private PIDController rightPIDController = new PIDController(RobotConstants.kPDriveVel,
                        RobotConstants.kIDriveVel, RobotConstants.kDDriveVel);

        private double rightSetPoint, leftSetPoint;

        private DifferentialDriveWheelSpeeds wheelSpeeds;

        public CenterToGenerator2BallSide(DriveBase driveBase, Shooter shooter, Intake intake, Conveyor conveyor, Indexer indexer) {
                this.driveBase = driveBase;
                this.shooter = shooter;
                this.intake = intake;
                this.conveyor = conveyor;
                this.indexer = indexer;
                trajectory = generateTrajectory();
                healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
                healthReqs.put(this.driveBase, HealthState.GREEN);
                healthReqs.put(this.shooter, HealthState.GREEN);
                healthReqs.put(this.intake, HealthState.GREEN);
                healthReqs.put(this.conveyor, HealthState.GREEN);
                healthReqs.put(this.indexer, HealthState.GREEN);
                addCommands(new StartShooter(shooter), 
                new RamseteCommand(trajectory, driveBase::getPose,
                                new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
                                                RobotConstants.kaVoltSecondsSquaredPerMeter),
                                RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController,
                                rightPIDController,
                                // RamseteCommand passes volts to the callback
                                driveBase::tankDriveVoltage, driveBase), 
                                new ParallelCommandGroup(new StopShooter(shooter), 
                                new RunIntake(0.5, intake),
                                new RunConveyor(conveyor), 
                                new RotateToIntakePosition(indexer)
                                )
                );

        }

        public Trajectory generateTrajectory() {
                driveBase.zeroHeading();
                driveBase.resetOdometry(new Pose2d(3.186, 4.296, Rotation2d.fromDegrees(0)));
                // Create a voltage constraint to ensure we don't accelerate too fast
                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
                                                RobotConstants.kaVoltSecondsSquaredPerMeter),
                                RobotConstants.kDriveKinematics, 10);

                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(RobotConstants.kMaxSpeedMetersPerSecond,
                                RobotConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(RobotConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(RobotConstants.kAutoPathConstraints)
                                                .addConstraint(autoVoltageConstraint);

                Trajectory trajectory = TrajectoryGenerator
                                .generateTrajectory(List.of(new Pose2d(3.186, 4.296, Rotation2d.fromDegrees(0)),
                                                new Pose2d(3.186, 4.296, Rotation2d.fromDegrees(39.748)),
                                                new Pose2d(6.151, 5.283, Rotation2d.fromDegrees(-64.678))), config);

                return trajectory;
        }

        @Override
        public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
                // maybe also check that NavX is there
                return healthReqs;
        }
}