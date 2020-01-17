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

        // System.out.println(pathnme);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(
             Paths.get(pathname + "\\straight.wpilib.json"));
         } catch (IOException e) {
           // TODO Auto-generated catch block
           e.printStackTrace();
         }

         for (int i = 0; i < trajectory.getStates().size(); i++) {
            System.out.println(trajectory.getStates().get(i));
         }

         assertEquals(1,0, 0.1);

    }
}
