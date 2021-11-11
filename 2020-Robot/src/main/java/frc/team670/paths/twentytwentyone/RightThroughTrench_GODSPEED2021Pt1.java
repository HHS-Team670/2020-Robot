package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.Logger;

/**
 * first part of godspeed trajectory
 * Trajectory starting from the right of the field in line with the trench then through the trench
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author meganchoy, elisevbp, tarini
 */
public class RightThroughTrench_GODSPEED2021Pt1 extends Path{

        public RightThroughTrench_GODSPEED2021Pt1(DriveBase driveBase) {
                super(

                        List.of(
                                //for now j goes forward 

                                // new Pose2d(3.944, -0.706, Rotation2d.fromDegrees(0)),
                                // new Pose2d(6.5, -0.706, Rotation2d.fromDegrees(0)),
                                // new Pose2d(7.8, -0.706, Rotation2d.fromDegrees(0))
                                new Pose2d(3.944, -0.706, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.427, -0.706, Rotation2d.fromDegrees(0))
                        ), 
                driveBase, RobotConstants.kAutoPathConstraints, RobotConstants.kMaxSpeedMetersPerSecond2, RobotConstants.kMaxAccelerationMetersPerSecondSquared2, RobotConstants.endVelocityMetersPerSecond2);
                //Logger.consoleLog("running godspeed part 1");
        }
}
