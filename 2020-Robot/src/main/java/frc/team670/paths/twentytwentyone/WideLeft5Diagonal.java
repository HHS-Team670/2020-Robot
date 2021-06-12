package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting to the left of the port (in perspective of the driver)
 * goes under the switch from the left side and intakes 5 balls in a diagonal line, starting closer to the 2 ball line (rather than 3 ball line)
 * front of robot starts on initiation line
 * wide bc enters switch from bottom left quadrant and exists from top left quadrant
 * google doc link: https://docs.google.com/document/d/1Sm1xBvk9HHvbotRF8uDx5V0EHe8H6kkJBm6AvSKBY1k/edit?usp=sharing
 * @author elisevbp
 */
public class WideLeft5Diagonal extends Path{

        public WideLeft5Diagonal (DriveBase driveBase) {
                super(
                        List.of(
                                new Pose2d(2.925, -3.915, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.208, -5.984, Rotation2d.fromDegrees(0)),
                                new Pose2d(6.715, -5.564, Rotation2d.fromDegrees(55.567)),
                                new Pose2d(6.492, -2.575, Rotation2d.fromDegrees(-87.583))



                                // new Pose2d(4.742, -5.159, Rotation2d.fromDegrees(16.172)),
                                // new Pose2d(5.787, -4.292, Rotation2d.fromDegrees(76.667))
                        ), 
                driveBase);
        }
}
