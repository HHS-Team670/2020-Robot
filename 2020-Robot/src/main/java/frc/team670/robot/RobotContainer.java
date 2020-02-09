/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.team670.robot.constants.OI;
import frc.team670.robot.dataCollection.sensors.ColorMatcher;
import frc.team670.robot.subsystems.ColorWheelSpinner;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private static List<MustangSubsystemBase> allSubsystems = new ArrayList<MustangSubsystemBase>();

  public static OI oi = new OI();

  public static Joystick operatorJoystick;
  private static DriveBase driveBase = new DriveBase();
  private static Shooter shooter = new Shooter();
  private static Turret turret = new Turret();
  private static Intake intake = new Intake();
  private static Indexer indexer = new Indexer();
  private final ColorWheelSpinner wheelSpinner = new ColorWheelSpinner();

  private Trajectory trajectory;
  private String pathname;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    addSubsystem(driveBase);
  }

  public static void addSubsystem(MustangSubsystemBase... subsystems) {
    for (MustangSubsystemBase m_subsystemBase : subsystems) {
      allSubsystems.add(m_subsystemBase);
    }
  }

  /**
   * Recalculates the health of all subsystems on the robot.
   */
  public static void checkSubsystemsHealth() {
    for (MustangSubsystemBase s : allSubsystems) {
      s.getHealth(true);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton toggleIntake = new JoystickButton(oi.getOperatorController(), 1);
    JoystickButton runIntakeOut = new JoystickButton(oi.getOperatorController(), 3);
    JoystickButton runIntakeIn = new JoystickButton(oi.getOperatorController(), 2);

    // toggleIntake.whenPressed(new toggleIntake()); // schedules the command when the button is pressed
    // runIntakeIn.whenHeld(new runIntakeIn());
    // runIntakeOut.whenHeld(new runIntakeOut());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    return null;
  }

  public static void teleopInit() {
    driveBase.setTeleopRampRate();
    driveBase.initDefaultCommand();
  }

  public static List<MustangSubsystemBase> getSubsystems(){
    return allSubsystems;
  }

}
