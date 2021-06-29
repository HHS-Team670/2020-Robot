package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.constants.RobotConstants;


/**
 * 2021 field
 * trajectory starting to the left of the port (in perspective of the driver) and going under the switch to pick up 3 balls in a line
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author megchoy
 */
public class Left3Line extends Path{

        public Left3Line(DriveBase driveBase) {
                super(
                        List.of(
                                //Starting position: line up angle of the robot to the generator and in front of the three ball line, 
                                // move back straight until initiation line. 
                                // Path: just goes straight forward to collect the three balls in line
                                // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                // new Pose2d(3.7, 0, Rotation2d.fromDegrees(0))

                                new Pose2d(3.201, -4.42, Rotation2d.fromDegrees(0)),
                                new Pose2d(6.671, -3.242, Rotation2d.fromDegrees(21.3))
                        ), 
                driveBase, RobotConstants.kAutoPathConstraintsCenter3Line, RobotConstants.kMaxSpeedMetersPerSecond3, RobotConstants.kMaxAccelerationMetersPerSecondSquared3, RobotConstants.endVelocityMetersPerSecond);
        }
}
