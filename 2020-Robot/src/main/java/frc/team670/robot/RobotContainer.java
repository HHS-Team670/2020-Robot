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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.LEDSubsystem;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Climber;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.MustangScheduler;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.commands.auton.ShootFromAngleThenTimeDrive;
import frc.team670.robot.commands.auton.ToTrenchRunAndShoot;
import frc.team670.robot.commands.auton.baseline.ShootThenBack;
import frc.team670.robot.commands.climb.ExtendClimber;
import frc.team670.robot.commands.climb.HookOnBar;
import frc.team670.robot.commands.climb.Climb;
import frc.team670.robot.commands.indexer.RotateToNextChamber;
import frc.team670.robot.commands.indexer.SendOneBallToShoot;
import frc.team670.robot.commands.indexer.StopIntaking;
import frc.team670.robot.commands.indexer.TogglePusher;
import frc.team670.robot.commands.indexer.ToggleUpdraw;
import frc.team670.robot.commands.indexer.UnjamIndexer;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.intake.ReverseIntakeConveyor;
import frc.team670.robot.commands.intake.RunConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.intake.ToggleIntake;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.routines.RotateIndexerToUptakeThenShoot;
import frc.team670.robot.commands.shooter.SetRPMAdjuster;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopUpdraw;
import frc.team670.robot.commands.shooter.ToggleShooter;
import frc.team670.robot.commands.turret.AutoRotate;
import frc.team670.robot.commands.turret.GetLatestDataAndAlignTurret;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.commands.turret.RotateToHome;
import frc.team670.robot.commands.turret.RotateTurret;
import frc.team670.robot.commands.turret.ZeroTurret;
import frc.team670.robot.commands.vision.GetVisionData;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.XboxButtons;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;
import edu.wpi.first.wpilibj.geometry.Pose2d;

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
  private static LEDSubsystem fancyLights = new LEDSubsystem(RobotMap.LEFT_SIDE_LEDS_PWM, 150);

  private static MustangCoprocessor coprocessor = new MustangCoprocessor();

  private static OI oi = new OI(driveBase, intake, conveyor, indexer, shooter, climber, turret, coprocessor);

  private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret, coprocessor);

  private static JoystickButton toggleIntake = new JoystickButton(oi.getOperatorController(), 1);
  private static JoystickButton runIntakeIn = new JoystickButton(oi.getOperatorController(), 3);
  private static JoystickButton runIntakeOut = new JoystickButton(oi.getOperatorController(), 5);
  private static JoystickButton toggleShooter = new JoystickButton(oi.getOperatorController(), 6);
  private static JoystickButton toggleUpdraw = new JoystickButton(oi.getOperatorController(), 2);
  private static JoystickButton rotateIndexerBackwards = new JoystickButton(oi.getOperatorController(), 9);
  private static JoystickButton togglePusher = new JoystickButton(oi.getOperatorController(), 7);
  private static JoystickButton extendClimb = new JoystickButton(oi.getOperatorController(), 11);
  private static JoystickButton retractClimb = new JoystickButton(oi.getOperatorController(), 12);
  private static JoystickButton turnToNextIndexer = new JoystickButton(oi.getOperatorController(), 10);
  private static JoystickButton zeroTurret = new JoystickButton(oi.getOperatorController(), 8);
  
  //xboxButtons
  private static JoystickButton xboxVision = new JoystickButton(oi.getDriverController(), XboxButtons.A);
  private static JoystickButton xboxIncreaseSpeed = new JoystickButton(oi.getDriverController(), XboxButtons.B);
  private static JoystickButton xboxDecreaseSpeed = new JoystickButton(oi.getDriverController(), XboxButtons.X);
  private static JoystickButton xboxToggleShooter = new JoystickButton(oi.getDriverController(), XboxButtons.Y);
  private static JoystickButton xboxRaiseClimber = new JoystickButton(oi.getDriverController(), XboxButtons.LEFT_JOYSTICK_BUTTON);
  private static JoystickButton xboxLowerClimber = new JoystickButton(oi.getDriverController(), XboxButtons.RIGHT_JOYSTICK_BUTTON);


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
      s.pushHealthToDashboard();
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
    toggleIntake.whenPressed(new ToggleIntake(intake));
    runIntakeIn.whenPressed(new IntakeBallToIndexer(intake, conveyor, indexer));
    runIntakeIn.whenReleased(new StopIntaking(intake, conveyor, indexer));
    runIntakeOut.toggleWhenPressed((new ReverseIntakeConveyor(intake, conveyor)));
    toggleShooter.toggleWhenPressed(new ToggleShooter(shooter, driveBase));
    toggleUpdraw.toggleWhenPressed(new ToggleUpdraw(indexer));
    rotateIndexerBackwards.whenHeld(new UnjamIndexer(indexer));
    togglePusher.whenHeld(new TogglePusher(indexer));
    extendClimb.whenPressed(new ExtendClimber(climber));
    retractClimb.whenPressed(new Climb(climber));
    turnToNextIndexer.whenPressed(new RotateToNextChamber(indexer, true));
    zeroTurret.whenPressed(new RotateToAngle(turret, 0));

    xboxVision.whenPressed(new GetLatestDataAndAlignTurret(turret, driveBase, coprocessor));
    xboxIncreaseSpeed.whenPressed(new SetRPMAdjuster(100, shooter));
    // xboxRunIntakeIn.whenPressed(new IntakeBallToIndexer(intake, conveyor, indexer));
    xboxDecreaseSpeed.whenPressed(new SetRPMAdjuster(-100, shooter)); //StopIntaking(intake, conveyor, indexer)
    xboxToggleShooter.toggleWhenPressed(new RotateIndexerToUptakeThenShoot(indexer, shooter, driveBase));
    // xboxLowerClimber.whenPressed(new Climb(climber));
    // xboxRaiseClimber.whenPressed(new ExtendClimber(climber));
  }

  public void robotInit() {
    // Turret should rotate automatically by default the whole time
    // MustangScheduler.getInstance().setDefaultCommand(turret, new AutoRotate(turret, coprocessor, driveBase));
    //REMOVE THE LINE BELOW---------TESTINGONLY
    // driveBase.resetOdometry(new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, 8.6868, Rotation2d.fromDegrees(180)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    Pose2d leftStart = new Pose2d(
      FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS + 
      (FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS - FieldConstants.EDGE_OF_BASELINE), 
      FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(0));
    Pose2d centerStart = new Pose2d(FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS, 
                 FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(180));
    Pose2d rightStart = new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, 
                 FieldConstants.EDGE_OF_BASELINE,
                 Rotation2d.fromDegrees(180));
    return
      //autoSelector.getSelectedRoutine();

      // CENTER: SHOOT THEN DRIVE BACK (AWAY FROM WALL)
      // new ShootFromAngleThenTimeDrive(centerStart, 0, 0, 0.3, driveBase, intake, conveyor, shooter, indexer, turret);

      // CENTER: SHOOT THEN DRIVE TOWARDS STATION WALL (PUSH)
      // new ShootFromAngleThenTimeDrive(centerStart, 0, 0, -0.7, driveBase, intake, conveyor, shooter, indexer, turret);

      // SHOOT THEN GO DOWN TRENCH
      new ToTrenchRunAndShoot(-25, driveBase, intake, conveyor, indexer, turret, shooter);
}


  public static void autonomousInit(){
    indexer.reset();
    // 3 balls, in set positions, preloaded for auto
    indexer.setChamberStatesForMatchInit();
    indexer.setRotatorMode(false); // indexer to brake mode
    if (!turret.hasZeroed()) { // only zero indexer if needed
      MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }
  }

  public static void teleopInit() {
    indexer.reset();
    indexer.setRotatorMode(false); // indexer to brake mode
    driveBase.resetOdometry(new Pose2d(FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS, 
    FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(180)));
    zeroSubsystemPositions();
    driveBase.setTeleopRampRate();
    driveBase.initDefaultCommand();
    if (!turret.hasZeroed()) {
      MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }
    turret.initDefaultCommand();
    coprocessor.turnOnLEDs();

    //REMOVE THE LINE BELOW ---- TESTINGOJYL
    // driveBase.resetOdometry(new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, 8.6868, Rotation2d.fromDegrees(180)));
    // MustangScheduler.getInstance().schedule(new RotateToAngle(turret, -180));
  }

  public static void disabled(){
    indexer.setRotatorMode(true); // indexer to coast mode
    coprocessor.turnOffLEDs();
  }

  public static List<MustangSubsystemBase> getSubsystems() {
    return allSubsystems;
  }

  public static Joystick getOperatorController() {
    return oi.getOperatorController();
  }

  public static void rumbleDriverController() {
    // oi.rumbleDriverController(0.7, 0.2);
    notifyDriverController(1.0, 0.3);
  }

  public static void rumbleDriverController(double power, double time){
    oi.rumbleDriverController(power, time);
  }

  public static void notifyDriverController(double power, double time){
    oi.notifyDriverController(power, time);
  }

  public static MustangController getDriverController() {
    return oi.getDriverController();
  }

  public static boolean isQuickTurnPressed() {
    return oi.isQuickTurnPressed();
  }

  public static void periodic() {
    fancyLights.periodic();
  }

}
