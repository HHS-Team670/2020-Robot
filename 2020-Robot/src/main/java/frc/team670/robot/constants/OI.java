package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.FlipDriveDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.climb.*;
import frc.team670.robot.commands.climb.ExtendClimber;
import frc.team670.robot.commands.indexer.*;
import frc.team670.robot.commands.indexer.ManualRunIndexer;
import frc.team670.robot.commands.*;
// import frc.team670.robot.commands.indexer.ShootAllBalls;
// import frc.team670.robot.commands.indexer.ShootBall;
// import frc.team670.robot.commands.indexer.ToggleUpdraw;
import frc.team670.robot.commands.intake.RunIntakeConveyor;
import frc.team670.robot.commands.intake.ToggleIntake;
import frc.team670.robot.commands.intake.*;
import frc.team670.robot.commands.routines.AutoIndex;
import frc.team670.robot.commands.routines.ShootAllBalls;
import frc.team670.robot.commands.shooter.*;
import frc.team670.robot.commands.shooter.ToggleShooter;
import frc.team670.robot.commands.turret.*;
import frc.team670.robot.commands.indexer.*;
import frc.team670.robot.commands.indexer.RunIndexer;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;
import frc.team670.mustanglib.commands.drive.teleop.*;

public class OI extends OIBase {

  private static MustangController driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
  private static Joystick operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);

  private static XKeys xkeys;

  // operator buttons
  private static JoystickButton runIntakeOut = new JoystickButton(getOperatorController(), 5);
  private static JoystickButton runIntakeIn = new JoystickButton(getOperatorController(), 3);
  private static JoystickButton runIndexer = new JoystickButton(getOperatorController(), 6);
  private static JoystickButton runUpdraw = new JoystickButton(getOperatorController(), 4);
  private static JoystickButton toggleIntake = new JoystickButton(getOperatorController(), 2);
  private static JoystickButton runConveyorIn = new JoystickButton(getOperatorController(), 7);
  private static JoystickButton runConveyorOut = new JoystickButton(getOperatorController(), 8);
  private static JoystickButton autoBallPickup = new JoystickButton(getOperatorController(), 9);
  private static JoystickButton toggleVision = new JoystickButton(getOperatorController(), 10);
  private static JoystickButton climbPrep = new JoystickButton(getOperatorController(), 11);
  private static JoystickButton climb = new JoystickButton(getOperatorController(), 12);
  

  // xbox buttons
  private static JoystickButton stop = new JoystickButton(getDriverController(), XboxButtons.X);
  private static JoystickButton stop2 = new JoystickButton(getDriverController(), XboxButtons.Y);
  
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
    int joy, oldjoy;
    if (!toggleVision.get()) {
      runIntakeOut.whenPressed(new RunIntake(true, intake));
      runIntakeIn.whenPressed(new RunIntake(false, intake));
      runIndexer.whenPressed(new RunIndexer(indexer, conveyor));
      runUpdraw.whenPressed(new ToggleUpdraw(indexer));
      toggleIntake.whenPressed(new ToggleIntake(intake));
      runConveyorIn.whenPressed(new RunConveyor(false, conveyor, indexer));
      runConveyorOut.whenPressed(new RunConveyor(true, conveyor, indexer));
      autoBallPickup.whenPressed(new ShootAllBalls(indexer, conveyor, shooter, vision));
      climbPrep.whenPressed(new DriveToBarAndPrepareClimb(drivebase, climber));
      climb.whenPressed(new ClimbMotion(climber));
      boolean trigger = operatorController.getTriggerPressed();
      if (trigger) {
        MustangScheduler.getInstance().schedule(new StartShooter(shooter));
        MustangScheduler.getInstance().schedule(new Shoot(shooter));
      }
      double thrott = operatorController.getThrottle();
      MustangScheduler.getInstance().schedule(new SetRPMAdjuster(thrott * 200.0, shooter));
      joy = operatorController.getPOV();
      MustangScheduler.getInstance().schedule(new RotateToAngle(turret, joy - 180.0));
    } else {
      runIntakeOut.whenPressed(new RunIntake(true, intake));
      runIntakeIn.whenPressed(new RunIntake(false, intake));
      runIndexer.whenPressed(new RunIndexer(indexer, conveyor));
      runUpdraw.whenPressed(new ToggleUpdraw(indexer));
      toggleIntake.whenPressed(new ToggleIntake(intake));
      runConveyorIn.whenPressed(new RunConveyor(false, conveyor, indexer));
      runConveyorOut.whenPressed(new RunConveyor(true, conveyor, indexer));
      autoBallPickup.whenPressed(new ShootAllBalls(indexer, conveyor, shooter, vision));
      climbPrep.whenPressed(new DriveToBarAndPrepareClimb(drivebase, climber));
      climb.whenPressed(new ClimbMotion(climber));
      boolean trigger = operatorController.getTriggerPressed();
      if (trigger) {
        MustangScheduler.getInstance().schedule(new VisionShooter(shooter, vision));
        MustangScheduler.getInstance().schedule(new Shoot(shooter));
      }
      double thrott = operatorController.getThrottle();
      MustangScheduler.getInstance().schedule(new SetRPMAdjuster(thrott * 200.0, shooter));
      oldjoy = joy;
      joy = operatorController.getPOV();
      if (oldjoy != joy) {
        MustangScheduler.getInstance().schedule(new GetLatestDataAndAlignTurret(turret, drivebase, coprocessor));
      }
    }

    MustangScheduler.getInstance().schedule(new XboxCurvatureDrive(drivebase, driverController));
    stop.whenPressed(new CancelAllCommands());
    stop2.whenPressed(new CancelAllCommands());



    

    xkeys = new XKeys(drivebase, intake, conveyor, indexer, shooter, climber, turret, vision);
  }
}