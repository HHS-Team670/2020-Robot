/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths.left;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Trajectory starting on the line near the opponent's loading station (robot facing towards your own driverstation)
 * and facing the 3 Power Cells under the middle of the generator.
 * 
 * @author meganchoy, ctychen
 */
public class LeftToGenerator3BallMidPath extends Path{

    public LeftToGenerator3BallMidPath(DriveBase driveBase) {
        super(
            List.of(
                new Pose2d(3.186, 1, Rotation2d.fromDegrees(0)),
                new Pose2d(3.186, 1, Rotation2d.fromDegrees(65.511)),
                new Pose2d(5.687, 3.6, Rotation2d.fromDegrees(18))
            ),
        driveBase);
    }
}