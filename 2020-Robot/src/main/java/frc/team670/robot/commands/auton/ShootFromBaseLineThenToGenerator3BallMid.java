/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.Path;
import frc.team670.paths.center.CenterToGenerator3BallMidPath;
import frc.team670.paths.left.LeftToGenerator3BallMidPath;
import frc.team670.paths.right.RightToGenerator3BallMidPath;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.turret.RotateTurretWithVision;
import frc.team670.robot.commands.shooter.Shoot;
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
 * Autonomous routine starting with shooting from the initiation line (facing
 * towards your driver station), ending at (and hopefully intaking and indexing)
 * 3 Power Cells under the generator.
 * 
 * @author ctychen, meganchoy
 */
public class ShootFromBaseLineThenToGenerator3BallMid extends SequentialCommandGroup implements MustangCommand {

  private Path trajectory;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  private enum StartPosition {
    LEFT, CENTER, RIGHT;
  }

  public ShootFromBaseLineThenToGenerator3BallMid(StartPosition startPosition, DriveBase driveBase, Intake intake,
      Conveyor conveyor, Shooter shooter, Indexer indexer, Turret turret, MustangCoprocessor coprocessor) {
    if (startPosition == StartPosition.LEFT)
      trajectory = new LeftToGenerator3BallMidPath(driveBase);
    if (startPosition == StartPosition.CENTER)
      trajectory = new CenterToGenerator3BallMidPath(driveBase);
    if (startPosition == StartPosition.RIGHT)
      trajectory = new RightToGenerator3BallMidPath(driveBase);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(driveBase, HealthState.GREEN);
    healthReqs.put(shooter, HealthState.GREEN);
    healthReqs.put(intake, HealthState.GREEN);
    healthReqs.put(conveyor, HealthState.GREEN);
    healthReqs.put(indexer, HealthState.GREEN);
    healthReqs.put(turret, HealthState.GREEN);
    addCommands(
      
      new ParallelCommandGroup (
        new StartShooter(shooter),
        new RotateTurretWithVision(turret, coprocessor)
      ),
      
      new ParallelCommandGroup (
        new SendAllBalls(indexer), 
        new Shoot(shooter)        
      ),
      
      new ParallelCommandGroup (
        getTrajectoryFollowerCommand(trajectory, driveBase),
        new IntakeBallToIndexer(intake, conveyor, indexer)       
      )
    );
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // maybe also check that NavX is there
    return healthReqs;
  }
}