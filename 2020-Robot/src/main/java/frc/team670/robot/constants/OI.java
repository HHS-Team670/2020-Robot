package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.constants.OIBase;
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

  private MustangController driverController;
  private Joystick operatorController;

  private JoystickButton toggleReverseDrive;

  private XKeys xkeys;

  //operator buttons
  private JoystickButton toggleIntake = new JoystickButton(getOperatorController(), 1);
  private JoystickButton runIntakeIn = new JoystickButton(getOperatorController(), 3);
  private JoystickButton runIntakeOut = new JoystickButton(getOperatorController(), 5);
  private JoystickButton toggleShooter = new JoystickButton(getOperatorController(), 6);
  private JoystickButton toggleUpdraw = new JoystickButton(getOperatorController(), 2);
  private JoystickButton rotateIndexerBackwards = new JoystickButton(getOperatorController(), 9);
  private JoystickButton togglePusher = new JoystickButton(getOperatorController(), 7);
  private JoystickButton extendClimb = new JoystickButton(getOperatorController(), 11);
  private JoystickButton retractClimb = new JoystickButton(getOperatorController(), 12);
  private JoystickButton turnToNextIndexer = new JoystickButton(getOperatorController(), 10);
  private JoystickButton zeroTurret = new JoystickButton(getOperatorController(), 8);
  
  //xbox buttons
  private JoystickButton xboxVision = new JoystickButton(getDriverController(), XboxButtons.A);
  private JoystickButton xboxIncreaseSpeed = new JoystickButton(getDriverController(), XboxButtons.B);
  private JoystickButton xboxDecreaseSpeed = new JoystickButton(getDriverController(), XboxButtons.X);
  private JoystickButton xboxToggleShooter = new JoystickButton(getDriverController(), XboxButtons.Y);
  private JoystickButton xboxRaiseClimber = new JoystickButton(getDriverController(), XboxButtons.LEFT_JOYSTICK_BUTTON);
  private JoystickButton xboxLowerClimber = new JoystickButton(getDriverController(), XboxButtons.RIGHT_JOYSTICK_BUTTON);

  DriveBase drivebase;
  Intake intake;
  Conveyor conveyor;
  Indexer indexer;
  Shooter shooter;
  Climber climber;
  Turret turret;
  Vision vision;

  public OI(DriveBase drivebase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Climber climber, Turret turret, Vision vision) {
    super();

    this.drivebase = drivebase;
    this.intake = intake;
    this.conveyor = conveyor;
    this.indexer = indexer;
    this.shooter = shooter;
    this.climber = climber;
    this.turret = turret;
    this.vision = vision;

    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveDirection());
    operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys(drivebase, intake, conveyor, indexer, shooter, climber, turret, vision);
  }

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

  public MustangController getDriverController() {
    return driverController;
  }

  public Joystick getOperatorController() {
    return operatorController;
  }

  public void configureButtonBindings(){
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
  }

}