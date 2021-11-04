package frc.team670.robot.commands.auton;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.auton.left.*;
import frc.team670.robot.commands.auton.center.*;
import frc.team670.robot.commands.auton.right.*;
import frc.team670.robot.constants.FieldConstants;
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
     * @param driveBase the drivebase of the robot
     * @param intake the intake of the robot
     * @param conveyor the conveyor of the robot
     * @param shooter the shooter of the robot
     * @param indexer the indexer of the robot
     * @param turret the turret of the robot
     * @param coprocessor the raspberry pi
     */
    public AutoSelector(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Turret turret, Vision coprocessor){
        
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
      LeftShoot2BallSide(0),

      CenterShoot3BallSide(1),
      RightShootTrench(2),
      UNKNOWN(-1);

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

    public static enum StartPosition{
        LEFT, 
        CENTER, 
        RIGHT;
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
        //   Logger.consoleLog("value: %s" , value.getType());
        //   return this.selectedRoutine;
        // }
        // Number autoID = value.getNumber(-1);
        Logger.consoleLog("auton selector id: %s", autoID);
        this.selectedRoutine = AutoRoutine.getById((int)(autoID.intValue()));
        Logger.consoleLog("auton selector routine: %s", this.selectedRoutine);
        return this.selectedRoutine;
    }

    /**
     * 
     * @return the command corresponding to the autonomous routine selected by the driver
     */
    public MustangCommand getSelectedRoutine(){
        AutoRoutine result = select();
        Logger.consoleLog("Auton %s", result);

          switch(result) {
            case LeftShoot2BallSide:
              return new LeftShootMoveOffInitiation(driveBase, intake, conveyor, indexer, turret, shooter); 
            case CenterShoot3BallSide:
              return new CenterShootMoveOffInitiation(driveBase, intake, conveyor, indexer, turret, shooter, coprocessor);
            case RightShootTrench:
              return new RightShootTrench(driveBase, intake, conveyor, indexer, turret, shooter, coprocessor);
            
            default:
              return new RightShootTrench(driveBase, intake, conveyor, indexer, turret, shooter, coprocessor);

           
            //TODO: check start position! -> only left or do a case&enum for each of the 3 directions?
            // case SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_2_BALL_SIDE_LEFT:
            //   return new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_2_BALL_SIDE_CENTER:
            //   return new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_2_BALL_SIDE_RIGHT:
            //   return new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);

            // case SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_3_BALL_MID_LEFT:
            //   return new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_3_BALL_MID_CENTER:
            //   return new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_3_BALL_MID_RIGHT:
            //   return new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);

            // case SHOOT_FROM_BASELINE_THEN_TO_TRENCH_LEFT:
            //   return new ShootFromBaseLineThenToTrench(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case SHOOT_FROM_BASELINE_THEN_TO_TRENCH_CENTER:
            //   return new ShootFromBaseLineThenToTrench(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case SHOOT_FROM_BASELINE_THEN_TO_TRENCH_RIGHT:
            //   return new ShootFromBaseLineThenToTrench(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
              
            // case SHOOT_THEN_BACK:
            //   return new ShootThenForward(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case GENERATOR_2_BALL_SIDE_TO_TRENCH_THEN_SHOOT:
            //   return new Generator2BallSideToTrenchThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // case GENERATOR_3_BALL_MID_TO_GENERATOR_2_BALL_MID_THEN_SHOOT:
            //   return new Generator3BallMidToGenerator2BallMidThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
            // default:
            //   return new ShootThenForward(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
          }
    }

}