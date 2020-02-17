/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.team670.robot.constants.RobotConstants;

/**
 * Assuming the robot is already at the 2 power cells at the side of the generator, drive to through the trench,
 * and end facing the initiation line
 * 
 * @author meganchoy, ctychen
 */
public class Generator2BallSideToTrenchPath {

    public static Trajectory generateTrajectory() {
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
                .generateTrajectory(
                    List.of(
                        // new Pose2d(5.8, 5.445, Rotation2d.fromDegrees(-65)),
                        new Pose2d(7.138, 6.178, Rotation2d.fromDegrees(23.92)),
                        new Pose2d(7.99, 6.816, Rotation2d.fromDegrees(68.689)),
                        new Pose2d(5.555, 7.437, Rotation2d.fromDegrees(180))
                    ),
                config);

        return trajectory;
    }
}