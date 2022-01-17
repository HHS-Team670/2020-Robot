/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.paths.left.Left2Line;
import frc.team670.robot.commands.auton.AutoSelector;
import frc.team670.robot.commands.auton.AutoSelector.StartPosition;
import frc.team670.robot.commands.auton.center.CenterShootMoveOffInitiation;
import frc.team670.robot.commands.auton.left.LeftShootMoveOffInitiation;
import frc.team670.robot.commands.auton.right.RightShootTrench;
import frc.team670.robot.commands.turret.ZeroTurret;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;


public class RobotContainer extends RobotContainerBase 
{

  private static OI oi = new OI();

  int i = 0;

  private MustangCommand m_autonomousCommand;

  private static DriveBase driveBase = new DriveBase(getDriverController());
  private static Intake intake = new Intake();
  private static Conveyor conveyor = new Conveyor();
  private static Indexer indexer = new Indexer(conveyor);
  private static Vision vision = new Vision();
  private static LED leds = new LED(0, 97);
  private static Turret turret = new Turret(vision, leds);
  private static Shooter shooter = new Shooter(vision);
  private static Climber climber = new Climber();
  private static AutoSelector autoSelector =  new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret, vision);
  // private static AutoSelector autoSelector = new AutoSelector(driveBase, intake, conveyor, indexer, shooter, turret,
  //     vision);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    super();
    addSubsystem(driveBase, intake, conveyor, indexer, turret, shooter, climber, vision); //climber, vision);
    oi.configureButtonBindings(driveBase, intake, conveyor, indexer, turret, shooter, climber, vision); //climber, vision);
  }

  public void robotInit() 
  {
    leds.enable(); led.setRed();
    vision.turnOnLEDs();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public MustangCommand getAutonomousCommand() 
  {
    // MustangCommand autonCommand = autoSelector.getSelectedRoutine();
    // MustangCommand autonCommand = new LeftShoot2BallSide(driveBase, intake, conveyor, indexer, turret, shooter);
    // MustangCommand autonCommand = new CenterSho ot3BallSide(driveBase, intake, conveyor, indexer, turret, shooter, vision);
    MustangCommand autonCommand = new RightShootTrench(driveBase, intake, conveyor, indexer, turret, shooter, vision);
    Logger.consoleLog("autonCommand: %s", autonCommand);
    return autonCommand;
  }

  public void autonomousInit() 
  {
    indexer.reset();
    turret.setLimitSwitch(false);
    if (!turret.hasZeroed()) { // only zero turret if needed
      MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }
    m_autonomousCommand = getAutonomousCommand();
    if (m_autonomousCommand != null) {
      MustangScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }
clear
  public void teleopInit() 
  {
    shooter.stop();
    indexer.stopUpdraw();
    indexer.stop();
    indexer.reset();

    driveBase.resetOdometry(new Pose2d(FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS,
        FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(180)));

    driveBase.setTeleopRampRate();
    driveBase.initDefaultCommand();
    turret.setLimitSwitch(true);

    if(turret.checkHealth().equals(HealthState.RED))
    {
      led.setred();
    }

    if (!turret.hasZeroed())
    {
      MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }

    // turret.initDefaultCommand();
    vision.turnOnLEDs();
  }

  @Override
  public void disabled() {
    vision.turnOffLEDs();
  }

  public static Joystick getOperatorController() {
    return OI.getOperatorController();
  }

  public static void rumbleDriverController() {
    notifyDriverController(1.0, 0.3);
  }

  public static void rumbleDriverController(double power, double time) {
    oi.rumbleDriverController(power, time);
  }

  public static void notifyDriverController(double power, double time) {
    oi.rumbleDriverController(power, time);
  }

  public static MustangController getDriverController() {
    return OI.getDriverController();
  }

  public void periodic() {
   
  }

}
