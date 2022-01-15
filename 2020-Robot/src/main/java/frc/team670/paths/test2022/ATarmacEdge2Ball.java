package frc.team670.paths.test2022;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2022 field
 * trajectory starting at edge of A tarmac 
 * @author Tarini
 */
public class ATarmacEdge2Ball extends Path{

        public ATarmacEdge2Ball(DriveBase driveBase) {
                super(
                        List.of(
                                //new Pose2d(3.09, -2.628, Rotation2d.fromDegrees(0)),
                                //new Pose2d(2, -2.628, Rotation2d.fromDegrees(0))
                        ), 
                driveBase, RobotConstants.kAutoPathConstraints, true);
        }
}