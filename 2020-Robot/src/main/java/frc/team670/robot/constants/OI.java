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
import frc.team670.robot.commands.indexer.RotateToNextChamber;
import frc.team670.robot.commands.indexer.StopIntaking;
import frc.team670.robot.commands.indexer.TogglePusher;
import frc.team670.robot.commands.indexer.ToggleUpdraw;
import frc.team670.robot.commands.indexer.UnjamIndexer;
import frc.team670.robot.commands.intake.ReverseIntakeConveyor;
import frc.team670.robot.commands.intake.ToggleIntake;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.routines.RotateIndexerToUptakeThenShoot;
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
  private static JoystickButton runIntakeIn = new JoystickButton(getOperatorController(), 3);
  private static JoystickButton runIntakeOut = new JoystickButton(getOperatorController(), 5);
  private static JoystickButton toggleShooter = new JoystickButton(getOperatorController(), 6);
  private static JoystickButton toggleUpdraw = new JoystickButton(getOperatorController(), 2);
  private static JoystickButton rotateIndexerBackwards = new JoystickButton(getOperatorController(), 9);
  private static JoystickButton togglePusher = new JoystickButton(getOperatorController(), 7);
  private static JoystickButton extendClimb = new JoystickButton(getOperatorController(), 11);
  private static JoystickButton retractClimb = new JoystickButton(getOperatorController(), 12);
  private static JoystickButton turnToNextIndexer = new JoystickButton(getOperatorController(), 10);
  private static JoystickButton zeroTurret = new JoystickButton(getOperatorController(), 8);
  
  //xbox buttons
  private static JoystickButton xboxVision = new JoystickButton(getDriverController(), XboxButtons.A);
  private static JoystickButton xboxIncreaseSpeed = new JoystickButton(getDriverController(), XboxButtons.B);
  private static JoystickButton xboxDecreaseSpeed = new JoystickButton(getDriverController(), XboxButtons.X);
  private static JoystickButton xboxToggleShooter = new JoystickButton(getDriverController(), XboxButtons.Y);
  private static JoystickButton xboxRaiseClimber = new JoystickButton(getDriverController(), XboxButtons.LEFT_JOYSTICK_BUTTON);
  private static JoystickButton xboxLowerClimber = new JoystickButton(getDriverController(), XboxButtons.RIGHT_JOYSTICK_BUTTON);
  private static JoystickButton toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);

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

  public static JoystickButton getRunIntakeIn() {
    return runIntakeIn;
  }

  public static JoystickButton getToggleShooter(){
    return ToggleShooter;
  }

  public static JoystickButton getExtendClimb(){
    return extendClimb;
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
    runIntakeIn.whenPressed(new IntakeBallToIndexer(intake, conveyor, indexer));
    runIntakeIn.whenReleased(new StopIntaking(intake, conveyor, indexer));
    runIntakeOut.toggleWhenPressed((new ReverseIntakeConveyor(intake, conveyor)));
    toggleShooter.toggleWhenPressed(new ToggleShooter(shooter, drivebase));
    toggleUpdraw.toggleWhenPressed(new ToggleUpdraw(indexer));
    rotateIndexerBackwards.whenHeld(new UnjamIndexer(indexer));
    togglePusher.whenHeld(new TogglePusher(indexer));
    extendClimb.whenPressed(new ExtendClimber(climber));
    retractClimb.whenPressed(new Climb(climber));
    turnToNextIndexer.whenPressed(new RotateToNextChamber(indexer, true));
    zeroTurret.whenPressed(new RotateToAngle(turret, 0));

    xboxVision.whenPressed(new GetLatestDataAndAlignTurret(turret, drivebase, vision));
    xboxIncreaseSpeed.whenPressed(new SetRPMAdjuster(100, shooter));
    // xboxRunIntakeIn.whenPressed(new IntakeBallToIndexer(intake, conveyor, indexer));
    xboxDecreaseSpeed.whenPressed(new SetRPMAdjuster(-100, shooter)); //StopIntaking(intake, conveyor, indexer)
    xboxToggleShooter.toggleWhenPressed(new RotateIndexerToUptakeThenShoot(indexer, shooter, drivebase));
    // xboxLowerClimber.whenPressed(new Climb(climber));
    // xboxRaiseClimber.whenPressed(new ExtendClimber(climber));
    toggleReverseDrive.whenPressed(new FlipDriveDirection());

    xkeys = new XKeys(drivebase, intake, conveyor, indexer, shooter, climber, turret, vision);
  }

}