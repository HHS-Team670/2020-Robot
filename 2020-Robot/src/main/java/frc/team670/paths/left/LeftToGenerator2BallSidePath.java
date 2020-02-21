package frc.team670.paths.left;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.paths.Path;

/**
 * Trajectory starting on the line near the opponent's loading station (robot facing towards your own driverstation), 
 * and facing the 2 Power Cells under the generator near your trench side.
 * 
 * @author meganchoy, ctychen
 */
public class LeftToGenerator2BallSidePath extends Path{

        public LeftToGenerator2BallSidePath(DriveBase driveBase) {
                super(
                        List.of(
                        new Pose2d(3.186, 1, Rotation2d.fromDegrees(0)),
                        new Pose2d(3.186, 1, Rotation2d.fromDegrees(82.163)),
                        new Pose2d(5.8, 5.445, Rotation2d.fromDegrees(-65))
                        ),
                driveBase);
        }
}