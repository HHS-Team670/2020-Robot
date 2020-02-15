/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths.right;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Trajectory starting on the line near your power port and facing the 2 Power
 * Cells under the generator near your trench side.
 * 
 * @author meganchoy, ctychen
 */
public class RightToGenerator2BallSide {

    public static Trajectory generateTrajectory(DriveBase driveBase) {
        driveBase.zeroHeading();
        driveBase.resetOdometry(new Pose2d(3.201, 7.451, Rotation2d.fromDegrees(0)));
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
                new Pose2d(3.201, 7.451, Rotation2d.fromDegrees(0)), List.of(new Translation2d(4.995, 6.673)),
                new Pose2d(5.8, 5.445, Rotation2d.fromDegrees(-65)), config);

        return trajectory;

    }
}