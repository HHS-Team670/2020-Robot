/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import edu.wpi.first.wpilibj.Filesystem;

import org.junit.Test;


/**
 * Add your docs here.
 */
public class AutonPathsTest {

    Trajectory trajectory;
    String pathname;
    
    @Test
    public void testAutonPaths() {

        pathname = Filesystem.getDeployDirectory() + "";

        System.out.println(pathname);
        Path path = Paths.get(pathname + "/../src/main/deploy/straight.wpilib.json");

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
         } catch (IOException e) {
             throw new RuntimeException("path is " + path, e);
         }

         for (int i = 0; i < trajectory.getStates().size(); i++) {
            System.out.println(trajectory.getStates().get(i));
         }

    }
}
