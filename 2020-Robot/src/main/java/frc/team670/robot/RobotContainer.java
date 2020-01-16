/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.team670.robot.commands.ExampleCommand;
import frc.team670.robot.constants.RobotConstants;

import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team670.robot.dataCollection.sensors.ColorMatcher;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveBase driveBase = new DriveBase();
  public static ColorMatcher colorMatch;// = new ColorMatcher();


  public static OI oi;
  public static Shooter shooter;// = new Shooter(RobotMap.SHOOTER_ID_MAIN, RobotMap.SHOOTER_ID_FOLLWOER);

  private Trajectory trajectory;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(RobotConstants.ksVolts,
                                       RobotConstants.kvVoltSecondsPerMeter,
                                       RobotConstants.kaVoltSecondsSquaredPerMeter),
            RobotConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(RobotConstants.kMaxSpeedMetersPerSecond,
                             RobotConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(RobotConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

            try {
               trajectory = TrajectoryUtil.fromPathweaverJson(
                Paths.get("src\\main\\deploy\\LeftDS-Trench.wpilib.json"));
            } catch (IOException e) {
              // TODO Auto-generated catch block
              e.printStackTrace();
            }

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        driveBase::getPose,
        new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
        new SimpleMotorFeedforward(RobotConstants.ksVolts,
          RobotConstants.kvVoltSecondsPerMeter,
          RobotConstants.kaVoltSecondsSquaredPerMeter),
          RobotConstants.kDriveKinematics,
        driveBase::getWheelSpeeds,
        new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel, RobotConstants.kDDriveVel),
        new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel, RobotConstants.kDDriveVel),
        // RamseteCommand passes volts to the callback
        driveBase::tankDriveVoltage,
        driveBase
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveBase.tankDrive(0, 0));

  }
}
