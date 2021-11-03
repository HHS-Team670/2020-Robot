package frc.team670.paths.left;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting to the left of the port (in perspective of the driver) and going forwards towards port off init line
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author megchoy
 */
public class Left2Line extends Path{

        public Left2Line(DriveBase driveBase) {
                super(
                        List.of(
                                new Pose2d(3.09, -2.628, Rotation2d.fromDegrees(0)),
                                new Pose2d(1, -2.628, Rotation2d.fromDegrees(-30))
                        ), 
                driveBase, RobotConstants.kAutoPathConstraints, RobotConstants.kMaxSpeedMetersPerSecond2, RobotConstants.kMaxAccelerationMetersPerSecondSquared2, RobotConstants.endVelocityMetersPerSecond2, true);
        }
}