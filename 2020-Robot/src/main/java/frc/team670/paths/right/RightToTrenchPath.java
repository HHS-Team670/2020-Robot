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
 * Trajectory starting on the line near your power port (intake facing towards your own driver station)
 * and through the trench
 * 
 * @author meganchoy, ctychen
 */
public class RightToTrenchPath extends Path{

        public RightToTrenchPath(DriveBase driveBase){
                super(
                        List.of(
                                new Pose2d(3.178, 7.352, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.09, 7.45, Rotation2d.fromDegrees(0.708))
                        ),
                        driveBase);
        }
}
