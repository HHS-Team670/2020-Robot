/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths.center;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Trajectory starting from middle of the initiation line (facing towards your driver station) 
 * and going through the trench.
 * 
 * @author meganchoy, ctychen
 */
public class CenterToTrenchPath extends Path{

    public CenterToTrenchPath(DriveBase driveBase){
        super(
                List.of(
                    new Pose2d(3.194, 4.296, Rotation2d.fromDegrees(0)),
                    new Pose2d(6.363, 7.519, Rotation2d.fromDegrees(0)),
                    new Pose2d(8.008, 7.508, Rotation2d.fromDegrees(0))
                ),
            driveBase);
    }
}
