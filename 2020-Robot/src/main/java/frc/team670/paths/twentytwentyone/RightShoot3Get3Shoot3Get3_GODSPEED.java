package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Trajectory starting from the right of the field in line with the trench
 * Shoot 3, 
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 *
 * 
 * @author AkshatAdsule, EliseVbp
 */
public class RightShoot3Get3Shoot3Get3_GODSPEED extends Path{

        public RightShoot3Get3Shoot3Get3_GODSPEED(DriveBase driveBase) {
                super(
                        List.of(
                                //for now j goes forward 
                                new Pose2d(3.944, -0.705, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.386, -0.705, Rotation2d.fromDegrees(0))
                        ), 
                driveBase);
        }
}
