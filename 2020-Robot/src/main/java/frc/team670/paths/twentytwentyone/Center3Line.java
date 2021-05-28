package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.Logger;

/**
 * 2021 field
 * trajectory starting at center and going under the switch to pick up 3 balls in a line
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author elisevbp
 */
public class Center3Line extends Path{

        public Center3Line(DriveBase driveBase) {
                super(
                        List.of(
                                new Pose2d(5.724, 4.513, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.698, 4.487, Rotation2d.fromDegrees(21.3)),
                                new Pose2d(7.103, 5.033, Rotation2d.fromDegrees(21.3))
                        ), 
                driveBase);
        }
}
