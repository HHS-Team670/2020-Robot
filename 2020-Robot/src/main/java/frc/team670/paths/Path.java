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
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Generic representation of a path that the robot can drive.
 * 
 * @author ctychen
 */
public class Path {

    //gets left voltage constraint bc left and right are very similar (even w 2020 clim)
    private static final DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = getLeftAutoVoltageConstraint();
    private static final TrajectoryConfig CONFIG = getConfig();
    private Trajectory trajectory;
    private DriveBase driveBase; 
    private List<Pose2d> waypointsList;

    /**
     * Used to create a path object based on a list of way points and the drivebase
     * @param waypoints a list of waypoints
     * @param driveBase the drivebase which has to follow the path
     */
    public Path(List<Pose2d> waypoints, DriveBase driveBase) {
        this.driveBase = driveBase;
        this.waypointsList = waypoints;
        trajectoryFromWaypoints(waypoints);
    }

        /**
     * Used to create a path object based on a list of way points and the drivebase
     * slower trajectory to compensate while intaking
     * @param waypoints a list of waypoints
     * @param driveBase the drivebase which has to follow the path
     * @param kAutoPathConstraints 
     *
     */
    public Path(List<Pose2d> waypoints, DriveBase driveBase, DifferentialDriveKinematicsConstraint kAutoPathConstraints, double kMaxSpeedMetersPerSecond, double kMaxAccelerationMetersPerSecondSquared, double endVelocityMetersPerSecond, boolean reversed) {
        // Logger.consoleLog("slow trajectory");
        this.driveBase = driveBase;
        this.waypointsList = waypoints;
        TrajectoryConfig config = getConfig(kAutoPathConstraints, kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared, endVelocityMetersPerSecond, reversed);
        trajectoryFromWaypoints(waypoints, config);
    }

        /**
     * Used to create a path object based on a list of way points and the drivebase
     * slower trajectory to compensate while intaking
     * @param waypoints a list of waypoints
     * @param driveBase the drivebase which has to follow the path
     * @param kAutoPathConstraints 
     *
     */
    public Path(List<Pose2d> waypoints, DriveBase driveBase, DifferentialDriveKinematicsConstraint kAutoPathConstraints, boolean reversed) {
        this(waypoints, driveBase, kAutoPathConstraints, RobotConstants.kMaxSpeedMetersPerSecond2, RobotConstants.kMaxAccelerationMetersPerSecondSquared2, RobotConstants.endVelocityMetersPerSecond2, reversed);
    }

    private void trajectoryFromWaypoints(List<Pose2d> waypoints){
        this.trajectory = TrajectoryGenerator.generateTrajectory(waypoints, CONFIG);
    }

    //for the slower trajectory
    private void trajectoryFromWaypoints(List<Pose2d> waypoints, TrajectoryConfig config){
        this.trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    /**
     * Gets the starting pose from the waypoint list
     */
    public Pose2d getStartingPose(){
        return this.waypointsList.get(0);
    }

    /**
     * Gets the ending pose from the waypoints list
     */
    public Pose2d getEndingPose(){
        return this.waypointsList.get(waypointsList.size() - 1);
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


    private static DifferentialDriveVoltageConstraint getLeftAutoVoltageConstraint() {
        return new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotConstants.leftKsVolts,
                RobotConstants.leftKvVoltSecondsPerMeter, RobotConstants.leftKaVoltSecondsSquaredPerMeter),
                RobotConstants.kDriveKinematics, 10);
    }

    private static TrajectoryConfig getConfig(DifferentialDriveKinematicsConstraint kAutoPathConstraints, double kMaxSpeedMetersPerSecond, double kMaxAccelerationMetersPerSecondSquared, double endVelocityMetersPerSecond, boolean reversed) {
        return new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(RobotConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(kAutoPathConstraints)
                        .addConstraint(AUTO_VOLTAGE_CONSTRAINT)
                        .setReversed(reversed)
                        .setEndVelocity(endVelocityMetersPerSecond);
                    
    }

    private static TrajectoryConfig getConfig() {
        return new TrajectoryConfig(RobotConstants.kMaxSpeedMetersPerSecond,
                RobotConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(RobotConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(RobotConstants.kAutoPathConstraints).addConstraint(AUTO_VOLTAGE_CONSTRAINT);
    }

    /**
     * Used to get the trajectory
     */
    public Trajectory getTrajectory(){
        return this.trajectory;
    }
}