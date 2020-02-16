package frc.team670.paths.left;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.constants.RobotConstants;

/**
 * Trajectory starting on the line near the opponent's loading station (robot facing towards your own driverstation), 
 * and facing the 2 Power Cells under the generator near your trench side.
 * 
 * @author meganchoy, ctychen
 */
public class LeftToGenerator2BallSidePath {

    public static Trajectory generateTrajectory(DriveBase driveBase) {

        driveBase.zeroHeading();
        driveBase.resetOdometry(new Pose2d(3.186, 1, Rotation2d.fromDegrees(0)));
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
                        .addConstraint(RobotConstants.kAutoPathConstraints).addConstraint(autoVoltageConstraint);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(3.186, 1, Rotation2d.fromDegrees(0)),
                        new Pose2d(3.186, 1, Rotation2d.fromDegrees(82.163)),
                        new Pose2d(5.8, 5.445, Rotation2d.fromDegrees(-65))
                    ),

        config);

    return trajectory;

    }
}