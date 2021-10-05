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
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
// import frc.team670.mustanglib.dataCollection.sensors.Multiplexer;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.auton.AutoSelector;
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

public class RobotContainer extends RobotContainerBase {

  private static OI oi = new OI();

  int i = 0;

  private static Solenoid indexerPusherClimberDeploy = new
  Solenoid(RobotMap.PCMODULE, RobotMap.INDEXER_PUSHER_CLIMBER_DEPLOY);

  private static DriveBase driveBase = new DriveBase(getDriverController());
  private static Intake intake = new Intake();
  // private static Multiplexer multiplexer = new Multiplexer(RobotMap.INDEXER_MUL_PORT);
  private static Conveyor conveyor = new Conveyor();
  private static Indexer indexer = new Indexer(conveyor);
  private static Turret turret = new Turret();
  private static Shooter shooter = new Shooter();
  private static Climber climber = new Climber(indexerPusherClimberDeploy); // TODO: find solenoid
  private static Vision vision = new Vision();

  private static LEDSubsystem fancyLights = new LEDSubsystem(RobotMap.LEFT_SIDE_LEDS_PWM, 150);

  private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret,
      vision);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(driveBase, intake, conveyor, indexer, turret, shooter, climber, vision); //climber, vision);
    oi.configureButtonBindings(driveBase, intake, conveyor, indexer, turret, shooter, climber, vision); //climber, vision);
  }

  public void robotInit() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public MustangCommand getAutonomousCommand() {
    MustangCommand autonCommand = autoSelector.getSelectedRoutine();
    Logger.consoleLog("autonCommand: %s", autonCommand);
    return autonCommand;
  }

  public void autonomousInit() {
    // indexer.reset();
    // 3 balls, in set positions, preloaded for auto
    // indexer.setChamberStatesForMatchInit();
    // indexer.setRotatorMode(false); // indexer to brake mode
    if (!turret.hasZeroed()) { // only zero indexer if needed
      MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }
    // m_autonomousCommand = getAutonomousCommand();
    // if (m_autonomousCommand != null) {
    // MustangScheduler.getInstance().schedule(m_autonomousCommand);
    // }
  }

  public void teleopInit() {
    shooter.stop();
    indexer.stopUpdraw();
    indexer.stop();
    driveBase.resetOdometry(new Pose2d(FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS,
        FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(180)));
    driveBase.setTeleopRampRate();
    driveBase.initDefaultCommand();
    if (!turret.hasZeroed()) {
      MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }
    turret.initDefaultCommand();
    vision.turnOnLEDs();
  }

  public void disabled() {
    vision.turnOffLEDs();
  }

  public static Joystick getOperatorController() {
    return getOperatorController();
  }

  public static void rumbleDriverController() {
    // oi.rumbleDriverController(0.7, 0.2);
    notifyDriverController(1.0, 0.3);
  }

  public static void rumbleDriverController(double power, double time) {
    oi.rumbleDriverController(power, time);
  }

  public static void notifyDriverController(double power, double time) {
    oi.rumbleDriverController(power, time);
  }

  public static MustangController getDriverController() {
    return oi.getDriverController();
  }

  public void periodic() {
    fancyLights.periodic();
    if(i==5){
      // Logger.consoleLog("Sensor0: %s Sensor1: %s Sensor2: %s", multiplexer.getSensors().get(0).getDistance(), multiplexer.getSensors().get(1).getDistance(), multiplexer.getSensors().get(2).getDistance());
      i=0;
    }
    i++;


    // Logger.consoleLog("Indexer Dis Entrance: %d, Chamber 1: %d", indexer.entranceSensor.getDistance(), indexer.indexerSensorChamber1.getDistance());

  }

}
