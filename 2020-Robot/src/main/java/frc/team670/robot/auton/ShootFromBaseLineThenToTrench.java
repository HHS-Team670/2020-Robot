/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.center.CenterToTrenchPath;
import frc.team670.paths.left.LeftToTrenchPath;
import frc.team670.paths.right.RightToTrenchPath;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.turret.RotateTurretWithVision;
import frc.team670.robot.commands.shooter.Shoot;
import frc.team670.robot.constants.RobotConstants;
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

    private Trajectory trajectory;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private PIDController leftPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
            RobotConstants.kDDriveVel);
    private PIDController rightPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
            RobotConstants.kDDriveVel);

    private enum StartPosition {
        LEFT, CENTER, RIGHT;
    }

    public ShootFromBaseLineThenToTrench(StartPosition startPosition, DriveBase driveBase, Intake intake,
            Conveyor conveyor, Shooter shooter, Indexer indexer, Turret turret, MustangCoprocessor coprocessor) {

        if (startPosition == StartPosition.LEFT)
            trajectory = LeftToTrenchPath.generateTrajectory(driveBase);
        if (startPosition == StartPosition.CENTER)
            trajectory = CenterToTrenchPath.generateTrajectory(driveBase);
        if (startPosition == StartPosition.RIGHT)
            trajectory = RightToTrenchPath.generateTrajectory(driveBase);
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
                new RamseteCommand(trajectory, driveBase::getPose,
                        new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
                                RobotConstants.kaVoltSecondsSquaredPerMeter),
                        RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController,
                        rightPIDController,
                        // RamseteCommand passes volts to the callback
                        driveBase::tankDriveVoltage, driveBase));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // maybe also check that NavX is there
        return healthReqs;
    }
}