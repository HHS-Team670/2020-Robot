package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting at center and going under the switch to pick up 2 balls in a line
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author elisevbp
 */
public class Center2Line extends Path{

        public Center2Line(DriveBase driveBase) {
                super(
                        List.of(
                                new Pose2d(5.724, 4.513, Rotation2d.fromDegrees(0)),

                                //TODO: same degree as Center3Line so if the angle for Center3Line is modifying change this too! 
                                new Pose2d(5.776, 4.201, Rotation2d.fromDegrees(21.3)),
                                new Pose2d(6.999, 4.669, Rotation2d.fromDegrees(21.3))
                        ), 
                driveBase);
        }
}
