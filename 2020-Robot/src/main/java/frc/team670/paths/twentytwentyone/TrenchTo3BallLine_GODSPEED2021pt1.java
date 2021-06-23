package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * third part of godspeed trajectory
 * starts at trench then goes over the bar and under the switch
 * fits 2021 field
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 *
 * 
 * @author AkshatAdsule, EliseVbp
 */
public class TrenchTo3BallLine_GODSPEED2021pt1 extends Path{

        public TrenchTo3BallLine_GODSPEED2021pt1(DriveBase driveBase) {
                super(

                        List.of(
                                new Pose2d(7.8, -0.706, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.985, -2.804, Rotation2d.fromDegrees(-159.174))
                        ), 
                driveBase, RobotConstants.kAutoPathConstraintsIntaking, RobotConstants.kMaxSpeedMetersPerSecond2, RobotConstants.kMaxAccelerationMetersPerSecondSquared2, RobotConstants.endVelocityMetersPerSecond2);
        }
}
