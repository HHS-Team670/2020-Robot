/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.auton.twentytwentyone.ShootThen5Diagonal;
import frc.team670.robot.commands.auton.twentytwentyone.ShootThen3Line;
import frc.team670.robot.commands.auton.twentytwentyone.ShootThen2Line;
import frc.team670.robot.commands.auton.twentytwentyone.TrenchLoop3Line;
import frc.team670.robot.commands.auton.twentytwentyone.TrenchLoop2Line;
// import frc.team670.robot.commands.auton.ShootFromAngleThenTimeDrive;
// import frc.team670.robot.commands.auton.ToTrenchRunAndShoot;
// import frc.team670.robot.commands.auton.baseline.ShootThenBack;
import frc.team670.robot.commands.turret.ZeroTurret;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

public class RobotContainer extends RobotContainerBase{


  private static OI oi = new OI();

  private static Solenoid indexerPusherClimberDeploy = new Solenoid(RobotMap.PCMODULE, RobotMap.INDEXER_PUSHER_CLIMBER_DEPLOY);

  private static DriveBase driveBase = new DriveBase(oi.getDriverController());
  private static Intake intake = new Intake();
  private static Conveyor conveyor = new Conveyor();
  private static Indexer indexer = new Indexer(conveyor, indexerPusherClimberDeploy);
  private static Turret turret = new Turret();
  private static Shooter shooter = new Shooter();
  private static Climber climber = new Climber(indexerPusherClimberDeploy);
  private static LEDSubsystem fancyLights = new LEDSubsystem(RobotMap.LEFT_SIDE_LEDS_PWM, 150);

  private static Vision vision = new Vision();

  private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret, vision);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(driveBase, intake, conveyor, indexer, turret, shooter, climber, vision);
    oi.configureButtonBindings(driveBase, intake, conveyor, indexer, turret, shooter, climber, vision);
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

  public void robotInit() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    // MustangCommand autonCoxmmand = autoSelector.getSelectedRoutine();
    // MustangCommand autonCommand = new TrenchLoop3Line(StartPosition.RIGHT, driveBase, intake, conveyor, indexer, turret, shooter);
    MustangCommand autonCommand = new ShootThen5Diagonal(StartPosition.CENTER, driveBase, intake, conveyor, indexer, turret, shooter);

    // Logger.consoleLog("autonCommand: %s", autonCommand);
    //MustangCommand autonCommand = new ShootThenForward(driveBase, intake, conveyor, shooter, indexer, turret, vision);
    return autonCommand;
  }


  public void autonomousInit(){
    indexer.reset();
    // 3 balls, in set positions, preloaded for auto
    indexer.setChamberStatesForMatchInit();
    indexer.setRotatorMode(false); // indexer to brake mode
    if (!turret.hasZeroed()) { // only zero indexer if needed
      MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }
    //  m_autonomousCommand = getAutonomousCommand();
    //  if (m_autonomousCommand != null) {
    //    MustangScheduler.getInstance().schedule(m_autonomousCommand);
    //  }
  }

  public void teleopInit() {
    shooter.stop();
    indexer.stopUpdraw();
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
    vision.turnOnLEDs();
  }

  public void disabled(){
    indexer.setRotatorMode(true); // indexer to coast mode
    vision.turnOffLEDs();
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
    oi.rumbleDriverController(power, time);
  }

  public static MustangController getDriverController() {
    return oi.getDriverController();
  }

  public void periodic() {
    SmartDashboard.putNumber("heading", driveBase.getHeading());
    fancyLights.periodic();
  }

}
