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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Generic representation of a path that the robot can drive.
 * 
 * @author ctychen
 */
public class Path {

    private static final DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = getAutoVoltageConstraint();
    private static final TrajectoryConfig CONFIG = getConfig();
    private Trajectory trajectory;
    private DriveBase driveBase;

    public Path(List<Pose2d> waypoints, DriveBase driveBase) {
        this.driveBase = driveBase;
        trajectoryFromWaypoints(waypoints);
    }

    private void trajectoryFromWaypoints(List<Pose2d> waypoints){
        this.trajectory = TrajectoryGenerator.generateTrajectory(waypoints, CONFIG);
    }

    /**
     * Zeros the heading and resets the pose to the drivebase's current pose. 
     * This shouldn't be called often -- should only be called at autonomous init.
     */
    public void reset() {
        driveBase.zeroHeading();
        // Resets odometry to the drivebase's current pose
        driveBase.resetOdometry(driveBase.getPose());
    }

    private static DifferentialDriveVoltageConstraint getAutoVoltageConstraint() {
        return new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.ksVolts,
                RobotConstants.kvVoltSecondsPerMeter, RobotConstants.kaVoltSecondsSquaredPerMeter),
                RobotConstants.kDriveKinematics, 10);
    }

    private static TrajectoryConfig getConfig() {
        return new TrajectoryConfig(RobotConstants.kMaxSpeedMetersPerSecond,
                RobotConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(RobotConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(RobotConstants.kAutoPathConstraints).addConstraint(AUTO_VOLTAGE_CONSTRAINT);
    }

    public Trajectory getTrajectory(){
        return this.trajectory;
    }
}