package frc.team670.robot.commands.auton;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.auton.center.CenterShootMoveOffInitiation;
import frc.team670.robot.commands.auton.left.LeftShootMoveOffInitiation;
import frc.team670.robot.commands.auton.right.RightShootTrench;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

/**
 * Selects an autonomous routine to run based on choice from driver
 */
public class AutoSelector {

  private static NetworkTableInstance instance;
  private static NetworkTable table;

  private DriveBase driveBase;
  private Intake intake;
  private Conveyor conveyor;
  private Indexer indexer;
  private Shooter shooter;
  private Turret turret;
  private Vision coprocessor;

  private Timer timer;

  AutoRoutine selectedRoutine = AutoRoutine.UNKNOWN;

  /**
   * Initializes this command from the given parameters
   * 
   * @param driveBase   the drivebase of the robot
   * @param intake      the intake of the robot
   * @param conveyor    the conveyor of the robot
   * @param shooter     the shooter of the robot
   * @param indexer     the indexer of the robot
   * @param turret      the turret of the robot
   * @param coprocessor the raspberry pi
   */
  public AutoSelector(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter,
      Turret turret, Vision coprocessor) {

    instance = NetworkTableInstance.getDefault();
    table = instance.getTable("SmartDashboard");

    this.driveBase = driveBase;
    this.intake = intake;
    this.conveyor = conveyor;
    this.indexer = indexer;
    this.shooter = shooter;
    this.turret = turret;
    this.coprocessor = coprocessor;

    timer = new Timer();
  }

  public static enum AutoRoutine {
    LeftShoot2BallSide(0), CenterShoot3BallSide(1), RightShootTrench(2), UNKNOWN(-1);

    private final int ID;

    AutoRoutine(int id) {
      ID = id;
    }

    public int getID() {
      return this.ID;
    }

    public static AutoRoutine getById(int id) {
      for (AutoRoutine e : values()) {
        if (e.getID() == id)
          return e;
      }
      return UNKNOWN;
    }

  }

  public static enum StartPosition {
    LEFT, CENTER, RIGHT;
  }

  /**
   * Gets the value of the enum for auto routines based on an int input from the
   * driver dashboard.
   * 
   * @return
   */
  public AutoRoutine select() {
    Number autoID = SmartDashboard.getNumber("auton-chooser", -1);
    timer.start();
    while (autoID.intValue() == -1) {
      autoID = SmartDashboard.getNumber("auton-chooser", -1);
      Logger.consoleLog("trying to get different value");
      if (timer.hasPeriodPassed(5))
        break;
    }
    // NetworkTableEntry value = table.getEntry("auton-chooser");
    // if (value.getType() != NetworkTableType.kDouble) {
    // Logger.consoleLog("value: %s" , value.getType());
    // return this.selectedRoutine;
    // }
    // Number autoID = value.getNumber(-1);
    Logger.consoleLog("auton selector id: %s", autoID);
    this.selectedRoutine = AutoRoutine.getById((int) (autoID.intValue()));
    Logger.consoleLog("auton selector routine: %s", this.selectedRoutine);
    return this.selectedRoutine;
  }

  /**
   * @return the command corresponding to the autonomous routine selected by the driver
   */
  public MustangCommand getSelectedRoutine() {
    AutoRoutine result = select();
    Logger.consoleLog("Auton %s", result);

    switch (result) {
      case LeftShoot2BallSide:
        return new LeftShootMoveOffInitiation(driveBase, intake, conveyor, indexer, turret, shooter);
      case CenterShoot3BallSide:
        return new CenterShootMoveOffInitiation(driveBase, intake, conveyor, indexer, turret, shooter, coprocessor);
      case RightShootTrench:
        return new RightShootTrench(driveBase, intake, conveyor, indexer, turret, shooter, coprocessor);
      default:
        return new RightShootTrench(driveBase, intake, conveyor, indexer, turret, shooter, coprocessor);
    }
  }

}