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
 * Assuming the robot is already at the 3 power cells under the generator, drives around the generator to the
 * 2 power cells on the side of the generator, near to the trench
 * 
 * @author meganchoy, ctychen
 */
public class Generator3BallMidToGenerator2BallSide {

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
                        // The point where the robot should already be at
                        // new Pose2d(5.687, 3.6, Rotation2d.fromDegrees(18)),
                        new Pose2d(5.073, 5.266, Rotation2d.fromDegrees(89.081)),
                        new Pose2d(5.8, 5.445, Rotation2d.fromDegrees(-65))
                    ),
                    config);

        return trajectory;
    }

}