/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auton.baseline;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.Path;
import frc.team670.paths.center.CenterToTrenchPath;
import frc.team670.paths.left.LeftToTrenchPath;
import frc.team670.paths.right.RightToTrenchPath;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
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
 * towards your driver station), going through the trench, and ending near the
 * control panel.
 * 
 * @author ctychen, meganchoy
 */
public class ShootFromBaseLineThenToTrench extends SequentialCommandGroup implements MustangCommand {

    private Path trajectory;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public ShootFromBaseLineThenToTrench(StartPosition startPosition, DriveBase driveBase, Intake intake,
            Conveyor conveyor, Shooter shooter, Indexer indexer, Turret turret, MustangCoprocessor coprocessor) {

        if (startPosition == StartPosition.LEFT)
            trajectory = new LeftToTrenchPath(driveBase);
        if (startPosition == StartPosition.CENTER)
            trajectory = new CenterToTrenchPath(driveBase);
        if (startPosition == StartPosition.RIGHT)
            trajectory = new RightToTrenchPath(driveBase);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(turret, HealthState.GREEN);
        addCommands(
                // Get shooter up to speed and aim
                new ParallelCommandGroup(
                    new StartShooter(shooter), 
                    new RotateTurretWithVision(turret, coprocessor)
                ),
                // Roll intake out and shoot
                new ParallelCommandGroup(
                    new Shoot(shooter), 
                    new SendAllBalls(indexer),
                    new IntakeBallToIndexer(intake, conveyor, indexer)
                ),
                getTrajectoryFollowerCommand(trajectory, driveBase)
        );
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // maybe also check that NavX is there
        return healthReqs;
    }
}