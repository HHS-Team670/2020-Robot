/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auton.generator;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.Generator3BallMidToGenerator2BallSidePath;
import frc.team670.paths.Path;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.commands.turret.RotateTurret;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

/**
 * Autonomous routine starting in front of the 2 power cells under the
 * generator, then towards the control panel, straight go through the trench,
 * and shoot by the initiation line
 * 
 * @author ctychen, meganchoy
 */
public class Generator2BallSideToTrenchThenShoot extends SequentialCommandGroup implements MustangCommand {

  private Path trajectory;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public Generator2BallSideToTrenchThenShoot(DriveBase driveBase, Intake intake, Conveyor conveyor,
      Shooter shooter, Indexer indexer, Turret turret, MustangCoprocessor coprocessor) {

    trajectory = new Generator3BallMidToGenerator2BallSidePath(driveBase);

    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(driveBase, HealthState.GREEN);
    healthReqs.put(shooter, HealthState.GREEN);
    healthReqs.put(intake, HealthState.GREEN);
    healthReqs.put(conveyor, HealthState.GREEN);
    healthReqs.put(indexer, HealthState.GREEN);
    healthReqs.put(turret, HealthState.GREEN);
    addCommands(
        // Intake and Shooter is already running
        getTrajectoryFollowerCommand(trajectory, driveBase),

        new ParallelCommandGroup(
          new Shoot(shooter), 
          new RotateTurret(turret, driveBase, coprocessor)
          ),
        new SendAllBalls(indexer));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // maybe also check that NavX is there
    return healthReqs;
  }
}