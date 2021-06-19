package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * fourth part of godspeed trajectory
 * goes under switch to intake 3 balls in a row
 * fits 2021 field
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 *
 * 
 * @author AkshatAdsule, EliseVbp
 */
public class TrenchTo3BallLine_GODSPEED2021pt2 extends Path{

        public TrenchTo3BallLine_GODSPEED2021pt2(DriveBase driveBase) {
                super(

                        List.of(
                                new Pose2d(7.985, -2.804, Rotation2d.fromDegrees(-159.174)),
                                new Pose2d(6.453, -3.441, Rotation2d.fromDegrees(-159.174))
                        ), 
                driveBase, RobotConstants.kAutoPathConstraintsIntaking);
        }
}
