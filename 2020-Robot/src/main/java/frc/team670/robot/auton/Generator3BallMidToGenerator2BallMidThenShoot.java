/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.Generator3BallMidToGenerator2BallSidePath;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.indexer.SendAllBalls;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;

/**
 * Autonomous routine starting in front of the 3 power cells under the generator, then to the
 * 2 power cells on the side of the generator, near the trench, and shooting from there
 * 
 * @author ctychen, meganchoy
 */
public class Generator3BallMidToGenerator2BallMidThenShoot extends SequentialCommandGroup implements MustangCommand {

  private DriveBase driveBase;
  private Shooter shooter;
  private Intake intake;
  private Conveyor conveyor;
  private Indexer indexer;
  private Trajectory trajectory;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  private PIDController leftPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
      RobotConstants.kDDriveVel);
  private PIDController rightPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
      RobotConstants.kDDriveVel);

  public Generator3BallMidToGenerator2BallMidThenShoot(DriveBase driveBase, Intake intake,
      Conveyor conveyor, Shooter shooter, Indexer indexer) {
    this.driveBase = driveBase;
    this.shooter = shooter;
    this.intake = intake;
    this.conveyor = conveyor;
    this.indexer = indexer;

    trajectory = Generator3BallMidToGenerator2BallSidePath.generateTrajectory();

    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(this.driveBase, HealthState.GREEN);
    healthReqs.put(this.shooter, HealthState.GREEN);
    healthReqs.put(this.intake, HealthState.GREEN);
    healthReqs.put(this.conveyor, HealthState.GREEN);
    healthReqs.put(this.indexer, HealthState.GREEN);
    addCommands(
        // Intake is already running (command previously called should be ShootFromBaseLineThenToGenerator3BallMid)
        // new ParallelCommandGroup(
        //   new RunIntake(0.5, intake), 
        //   new RunConveyor(conveyor),
        //   new RotateToIntakePosition(indexer)
        // ),
        new RamseteCommand(trajectory, driveBase::getPose,
            new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
              new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
              RobotConstants.kaVoltSecondsSquaredPerMeter),
              RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController, rightPIDController,
            // RamseteCommand passes volts to the callback
            driveBase::tankDriveVoltage, driveBase),
        new StartShooter(shooter),
        new SendAllBalls(indexer)
    );
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // maybe also check that NavX is there
    return healthReqs;
  }
}
