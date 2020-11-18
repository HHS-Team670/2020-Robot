/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths.right;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Trajectory starting on the line and to the right of your power port and facing the 3 Power Cells under
 * the middle of the generator.
 * 
 * @author meganchoy, ctychen
 */
public class RightToGenerator3BallMidPath extends Path{

    public RightToGenerator3BallMidPath(DriveBase driveBase){
        super(
            List.of(
                new Pose2d(1.005, 2.958, Rotation2d.fromDegrees(0)),
                new Pose2d(3.276, 4.283, Rotation2d.fromDegrees(-80.354)),
                new Pose2d(4.081, 5.679, Rotation2d.fromDegrees(22.255))
            ),
            driveBase);
    }
}
