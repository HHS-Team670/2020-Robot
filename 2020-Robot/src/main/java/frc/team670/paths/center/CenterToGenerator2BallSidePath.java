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
 * Trajectory starting from the middle of the initiation line (facing towards
 * your driver station) and facing 2 Power Cells under the generator near your
 * trench.
 * 
 * @author meganchoy, ctychen
 */
public class CenterToGenerator2BallSidePath extends Path{

        public CenterToGenerator2BallSidePath(DriveBase driveBase) {
                super(
                        List.of(
                                new Pose2d(3.186, 4.296, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.8, 5.445, Rotation2d.fromDegrees(-65))
                        ), 
                driveBase);
        }
}