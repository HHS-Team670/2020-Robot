package frc.team670.paths.right;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.constants.RobotConstants;

/** 
 * Trajectory starting from the right of the field in line with the trench then through the trench to the last ball (3 balls total)
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author meganchoy, elisevbp, tarini, rishabh b
 */
public class RightThroughTrench extends Path{

        public RightThroughTrench(DriveBase driveBase) {
                super(

                        List.of(
                                new Pose2d(3.944, -0.706, Rotation2d.fromDegrees(0)),
                                new Pose2d(6.5, -0.706, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.8, -0.706, Rotation2d.fromDegrees(0))
                        ), 
                driveBase, RobotConstants.kAutoPathConstraints, false);
        }
}