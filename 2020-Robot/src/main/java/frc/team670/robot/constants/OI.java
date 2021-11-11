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
import frc.team670.robot.commands.indexer.*;
import frc.team670.robot.commands.indexer.ManualRunIndexer;
import frc.team670.robot.commands.*;
// import frc.team670.robot.commands.indexer.ShootAllBalls;
// import frc.team670.robot.commands.indexer.ShootBall;
// import frc.team670.robot.commands.indexer.ToggleUpdraw;
import frc.team670.robot.commands.intake.RunIntakeConveyor;
import frc.team670.robot.commands.intake.ToggleIntake;
import frc.team670.robot.commands.intake.RunConveyor;
import frc.team670.robot.commands.routines.AutoIndex;
import frc.team670.robot.commands.routines.ShootAllBalls;
import frc.team670.robot.commands.shooter.*;
import frc.team670.robot.commands.shooter.ToggleShooter;
import frc.team670.robot.commands.turret.*;
import frc.team670.robot.commands.indexer.*;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static Joystick operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);

  private static XKeys xkeys;

  // operator buttons
  private static JoystickButton toggleIntake = new JoystickButton(getOperatorController(), 1);
  private static JoystickButton shoot = new JoystickButton(getOperatorController(), 4);
  private static JoystickButton conveyorSwitch = new JoystickButton(getOperatorController(), 3);
  private static JoystickButton indexerSwitch = new JoystickButton(getOperatorController(), 8);
  private static JoystickButton climber = new JoystickButton(getOperatorController(), 5);
  private static JoystickButton stopAll = new JoystickButton(getOperatorController(), 6);
  private static JoystickButton accelerateShooter = new JoystickButton(getOperatorController(), 7);
  private static JoystickButton turnTurret = new JoystickButton(getOperatorController(), 2);
  

  // xbox buttons
  private static JoystickButton xboxVision = new JoystickButton(getDriverController(), XboxButtons.A);
  private static JoystickButton stop = new JoystickButton(getDriverController(), XboxButtons.X);
  
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

  public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
    DriveBase drivebase = (DriveBase) subsystemBases[0];
    Intake intake = (Intake) subsystemBases[1];
    Conveyor conveyor = (Conveyor) subsystemBases[2];
    Indexer indexer = (Indexer) subsystemBases[3];
    Turret turret = (Turret) subsystemBases[4];
    Shooter shooter = (Shooter) subsystemBases[5];
    Climber climber = (Climber) subsystemBases[6];
    Vision vision = (Vision) subsystemBases[7];

    toggleIntake.whenPressed(new ToggleIntake(intake));
    shoot.whenPressed(new Shoot(shooter));
    conveyorSwitch.whenPressed(new RunConveyor(false, conveyor, indexer));
    this.climber.whenPressed(new Climb(climber));
    indexerSwitch.whenPressed(new RunIndexer(indexer, conveyor));
    stopAll.whenPressed(null);
    stop.whenPressed(null);
    accelerateShooter.whenPressed(new StartShooter(shooter));
    turnTurret.whenPressed(new AutoRotate(turret, vision, drivebase));


    

    

    xboxVision.whenPressed(new GetLatestDataAndAlignTurret(turret, drivebase, vision));
    

    xkeys = new XKeys(drivebase, intake, conveyor, indexer, shooter, climber, turret, vision);
  }
}