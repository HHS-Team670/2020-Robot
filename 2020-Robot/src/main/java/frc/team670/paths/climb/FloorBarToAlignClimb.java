package frc.team670.paths.climb;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Drives straight to position for extending the climber after aligning against the floor bars
 */
public class FloorBarToAlignClimb extends Path {

    private static final double BACK_WHEEL_TO_HOOK_METERS = 0.2032;

    public FloorBarToAlignClimb(DriveBase driveBase) {
        super(
            List.of(
                driveBase.getPose(),
                new Pose2d(
                        driveBase.getPose().getTranslation().getX(), 
                        driveBase.getPose().getTranslation().getY()
                                + FieldConstants.FLOOR_BAR_TO_CLIMBER_BAR_METERS - BACK_WHEEL_TO_HOOK_METERS,
                        Rotation2d.fromDegrees(0))
            ),
        driveBase);
    }

}