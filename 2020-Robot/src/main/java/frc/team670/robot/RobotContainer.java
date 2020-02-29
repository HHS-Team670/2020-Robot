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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.climber.Climber;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.MustangScheduler;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.commands.climb.ExtendClimber;
import frc.team670.robot.commands.climb.HookOnBar;
import frc.team670.robot.commands.climb.Climb;
import frc.team670.robot.commands.indexer.RotateToNextChamber;
import frc.team670.robot.commands.indexer.StopIntaking;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.routines.RotateIndexerToUptakeThenShoot;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.commands.turret.RotateTurret;

import frc.team670.robot.utils.MustangController;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;

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

  private static Solenoid indexerPusherClimberDeploy = new Solenoid(RobotMap.PCMODULE, RobotMap.INDEXER_PUSHER_CLIMBER_DEPLOY);

  private static DriveBase driveBase = new DriveBase();
  private static Intake intake = new Intake();
  private static Conveyor conveyor = new Conveyor();
  private static Indexer indexer = new Indexer(conveyor, indexerPusherClimberDeploy);
  private static Turret turret = new Turret();
  private static Shooter shooter = new Shooter();
  private static Climber climber = new Climber(indexerPusherClimberDeploy);

  private static MustangCoprocessor coprocessor = new MustangCoprocessor();

  private static OI oi = new OI(driveBase, intake, conveyor, indexer, shooter, climber, turret, coprocessor);

  private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret, coprocessor);

  private static JoystickButton toggleIntake = new JoystickButton(oi.getOperatorController(), 1);
  private static JoystickButton runIntakeIn = new JoystickButton(oi.getOperatorController(), 3);
  private static JoystickButton runIntakeOut = new JoystickButton(oi.getOperatorController(), 5);
  private static JoystickButton toggleShooter = new JoystickButton(oi.getOperatorController(), 6);
  private static JoystickButton sendOneBall = new JoystickButton(oi.getOperatorController(), 2);
  private static JoystickButton extendClimb = new JoystickButton(oi.getOperatorController(), 11);
  private static JoystickButton retractClimb = new JoystickButton(oi.getOperatorController(), 12);
  private static JoystickButton hook = new JoystickButton(oi.getOperatorController(), 10);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    addSubsystem(driveBase, intake, conveyor, indexer, shooter, climber, turret);
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
      // s.pushHealthToDashboard();
    }
  }

  /**
   * Resets subsystem points of reference.
   * Rotates the indexer to its zero position.
   */
  public static void zeroSubsystemPositions() {
    indexer.setEncoderPositionFromAbsolute();
  }

  public static void clearSubsystemSetpoints(){
    indexer.clearSetpoint();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    toggleIntake.whenPressed(new DeployIntake(!intake.isDeployed(), intake));
    runIntakeIn.whenPressed(new IntakeBallToIndexer(intake, conveyor, indexer));
    runIntakeIn.whenReleased(new StopIntaking(intake, conveyor, indexer));
    runIntakeOut.toggleWhenPressed(new RunIntake(false, intake));
    toggleShooter.toggleWhenPressed(new RotateIndexerToUptakeThenShoot(indexer, shooter));
    sendOneBall.whenHeld(new RotateToNextChamber(indexer));
    extendClimb.whenHeld(new ExtendClimber(climber));
    retractClimb.whenPressed(new Climb(climber));
    hook.whenPressed(new HookOnBar(climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    // return (MustangCommand)(new ParallelCommandGroup(
    //   new RotateToHome(turret),
    //   (Command)(autoSelector.getSelectedRoutine())
    // ));
    return new RotateToHome(turret);
  }

  public static void autonomousInit(){
    indexer.reset();
    // 3 balls, in set positions, preloaded for auto
    indexer.setChamberStatesForMatchInit();
  }

  public static void teleopInit() {
    indexer.reset();
    zeroSubsystemPositions();
    driveBase.setTeleopRampRate();
    driveBase.initDefaultCommand();
    MustangScheduler.getInstance().schedule(new RotateToHome(turret));
    // turret.initDefaultCommand();
  }

  public static void disabled(){
  }

  public static void teleopPeriodic() {
    // climber.test();
    toggleIntake.whenPressed(new DeployIntake(!intake.isDeployed(), intake));
    MustangScheduler.getInstance().run();
  }

  public static List<MustangSubsystemBase> getSubsystems() {
    return allSubsystems;
  }

  public static Joystick getOperatorController() {
    return oi.getOperatorController();
  }

  public static void rumbleDriverController() {
    oi.rumbleDriverController(0.7, 0.2);
  }

  public static MustangController getDriverController() {
    return oi.getDriverController();
  }

  public static boolean isQuickTurnPressed() {
    return oi.isQuickTurnPressed();
  }

}
