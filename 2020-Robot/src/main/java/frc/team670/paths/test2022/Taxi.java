package frc.team670.paths.test2022;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

public class Taxi extends Path{
    public Taxi(DriveBase driveBase) {
        super(List.of(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
            driveBase, RobotConstants.kAutoPathConstraints, true);
    }
}
