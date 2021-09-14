package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.climb.Climb;
import frc.team670.robot.commands.climb.ExtendClimber;
import frc.team670.robot.commands.indexer.ShootAllBalls;
import frc.team670.robot.commands.indexer.ShootBall;
import frc.team670.robot.commands.indexer.ToggleUpdraw;
import frc.team670.robot.commands.intake.RunIntakeConveyor;
import frc.team670.robot.commands.intake.ToggleIntake;
import frc.team670.robot.commands.shooter.SetRPMAdjuster;
import frc.team670.robot.commands.shooter.ToggleShooter;
import frc.team670.robot.commands.turret.GetLatestDataAndAlignTurret;
import frc.team670.robot.commands.turret.RotateToAngle;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

public class OI extends OIBase{

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static Joystick operatorController  = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);

  private static XKeys xkeys;

  //operator buttons
  private static JoystickButton toggleIntake = new JoystickButton(getOperatorController(), 1);
  private static JoystickButton runIntakeOut = new JoystickButton(getOperatorController(), 5);
  private static JoystickButton runIntakeIn = new JoystickButton(getOperatorController(), 3);
  // private static JoystickButton toggleUpdraw = new JoystickButton(getOperatorController(), 2);
  private static JoystickButton toggleShooter = new JoystickButton(getOperatorController(), 6);
  private static JoystickButton extendClimb = new JoystickButton(getOperatorController(), 11);
  private static JoystickButton retractClimb = new JoystickButton(getOperatorController(), 12);
  private static JoystickButton zeroTurret = new JoystickButton(getOperatorController(), 8);
  private static JoystickButton intakeBallToIndexer = new IntakeBallToIndexer(getOperatorController(), 2);
  // private static JoystickButton nextChamber = new JoystickButton(getOperatorController(), 2); // TODO: figure number
  
  //xbox buttons
  private static JoystickButton xboxVision = new JoystickButton(getDriverController(), XboxButtons.A);
  private static JoystickButton xboxIncreaseSpeed = new JoystickButton(getDriverController(), XboxButtons.B);
  private static JoystickButton xboxDecreaseSpeed = new JoystickButton(getDriverController(), XboxButtons.X);
  private static JoystickButton xboxRaiseClimber = new JoystickButton(getDriverController(), XboxButtons.LEFT_JOYSTICK_BUTTON);
  private static JoystickButton xboxLowerClimber = new JoystickButton(getDriverController(), XboxButtons.RIGHT_JOYSTICK_BUTTON);
  private static JoystickButton toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
  private static JoystickButton shootAllBalls = new JoystickButton(getDriverController(), XboxButtons.Y); // TODO: figure button
  private static JoystickButton shootBall = new JoystickButton(getDriverController(), XboxButtons.RIGHT_BUMPER); // TODO: figure button
 



  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }

  /**
   * Sets the rumble on the driver controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time  The time to rumble for in seconds
   */
  public void rumbleDriverController(double power, double time) {
    rumbleController(driverController, power, time);
  }

  public static MustangController getDriverController() {
    return driverController;
  }

  public static Joystick getOperatorController() {
    return operatorController;
  }

  public void configureButtonBindings(MustangSubsystemBase... subsystemBases){
    DriveBase drivebase = (DriveBase)subsystemBases[0];
    Intake intake = (Intake)subsystemBases[1];
    Conveyor conveyor = (Conveyor)subsystemBases[2];
    Indexer indexer = (Indexer)subsystemBases[3];
    Turret turret = (Turret)subsystemBases[4];
    Shooter shooter = (Shooter)subsystemBases[5];
    Climber climber = (Climber)subsystemBases[6];
    Vision vision = (Vision)subsystemBases[7];

    toggleIntake.whenPressed(new ToggleIntake(intake));
    runIntakeIn.toggleWhenPressed((new RunIntakeConveyor(intake, conveyor, false)));
    runIntakeOut.toggleWhenPressed((new RunIntakeConveyor(intake, conveyor, true)));
    toggleUpdraw.toggleWhenPressed(new ToggleUpdraw(indexer));
    toggleShooter.toggleWhenPressed(new ToggleShooter(shooter, drivebase));
    extendClimb.whenPressed(new ExtendClimber(climber));
    retractClimb.whenPressed(new Climb(climber));
    zeroTurret.whenPressed(new RotateToAngle(turret, 0));
    intakeBallToIndexer.whenPressed(new IntakeBallToIndexer(intake, conveyor, indexer));

    xboxVision.whenPressed(new GetLatestDataAndAlignTurret(turret, drivebase, vision));
    xboxIncreaseSpeed.whenPressed(new SetRPMAdjuster(100, shooter));
    xboxDecreaseSpeed.whenPressed(new SetRPMAdjuster(-100, shooter));
    xboxLowerClimber.whenPressed(new Climb(climber));
    xboxRaiseClimber.whenPressed(new ExtendClimber(climber));
    toggleReverseDrive.whenPressed(new FlipDriveDirection());
    shootBall.whenPressed(new ShootBall(indexer));
    shootAllBalls.whenPressed(new ShootAllBalls(indexer));

    xkeys = new XKeys(drivebase, intake, conveyor, indexer, shooter, climber, turret, vision);
  }

}