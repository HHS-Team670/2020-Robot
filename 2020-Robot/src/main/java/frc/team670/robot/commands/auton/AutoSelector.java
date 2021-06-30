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
import frc.team670.paths.twentytwentyone.RightThroughTrench_GODSPEED2021Pt1;
import frc.team670.robot.commands.auton.baseline.*;
import frc.team670.robot.commands.auton.twentytwentyone.*;
import frc.team670.robot.commands.auton.generator.*;
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

        // LEFT_TO_GENERATOR_2_BALL_SIDE(0), 
        // LEFT_TO_GENERATOR_3_BALL_SIDE(1), 
        // LEFT_TO_TRENCH(2),
        // LEFT_TO_GENERATOR_2_TO_TRENCH(3), 
        // LEFT_TO_GENERATOR_3_TO_2_BALL_SIDE(4),

        // CENTER_TO_GENERATOR_2_BALL_SIDE(5), 
        // CENTER_TO_GENERATOR_3_BALL_SIDE(6), 
        // CENTER_TO_TRENCH(7),
        // CENTER_TO_GENERATOR_2_TO_TRENCH(8), 
        // CENTER_TO_GENERATOR_3_TO_2_BALL_SIDE(9),

        // RIGHT_TO_GENERATOR_2_BALL_SIDE(10), 
        // RIGHT_TO_GENERATOR_3_BALL_SIDE(11), 
        // RIGHT_TO_TRENCH(12),
        // RIGHT_TO_GENERATOR_2_TO_TRENCH(13), 
        // RIGHT_TO_GENERATOR_3_TO_2_BALL_SIDE(14),

        //as of 5/11
        // LEFT_EMPTY_THEN_BACK(0),
        // LEFT_EMPTY_THEN_FRONT(1),
        
        // CENTER_EMPTY_THEN_BACK(2),
        // CENTER_EMPTY_THEN_FRONT(3),

        // RIGHT_EMPTY_THEN_BACK(4),
        // RIGHT_EMPTY_THEN_FRONT(5),

        // RIGHT_TO_TRENCH_SHOT(6),



        //baseline
        // SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_2_BALL_SIDE_LEFT(0),        
        // SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_2_BALL_SIDE_CENTER(1),
        // SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_2_BALL_SIDE_RIGHT(2),

        // SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_3_BALL_MID_LEFT(3),
        // SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_3_BALL_MID_CENTER(4),
        // SHOOT_FROM_BASELINE_THEN_TO_GENERATOR_3_BALL_MID_RIGHT(5),

        // SHOOT_FROM_BASELINE_THEN_TO_TRENCH_LEFT(6),
        // SHOOT_FROM_BASELINE_THEN_TO_TRENCH_CENTER(7),
        // SHOOT_FROM_BASELINE_THEN_TO_TRENCH_RIGHT(8),

        // SHOOT_THEN_BACK(9),

        // //generator
        // GENERATOR_2_BALL_SIDE_TO_TRENCH_THEN_SHOOT(10),
        // GENERATOR_3_BALL_MID_TO_GENERATOR_2_BALL_MID_THEN_SHOOT(11),

        // 2021 paths
        RightTrenchLoop3Line_GODSPEED(1),
        RightTrenchLoop2Line(2),
        CenterTrenchLoop3Line_GODSPEED(3),
        CenterTrenchLoop2Line_GODSPEED(4),
        Center3Line(5),
        Center2Line(6),
        Left3Line(7),
        Left2Line(8),
        CenterDiagonal(9),
        RightDiagonal(10),

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

    // private Pose2d leftStart = new Pose2d(
    //     FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS + 
    //     (FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS - FieldConstants.EDGE_OF_BASELINE), 
    //     FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(0));
    // private Pose2d centerStart = new Pose2d(FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS, 
    //                FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(180));
    // private Pose2d rightStart = new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, 
    //                FieldConstants.EDGE_OF_BASELINE,
    //                Rotation2d.fromDegrees(180));

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
     * @return the delay between shooting and moving for this command as entered by the driver
     */
    private double getWaitTime(){
        // TODO
        return 0;
    }

    /**
     * 
     * @return the command corresponding to the autonomous routine selected by the driver
     */
    public MustangCommand getSelectedRoutine(){
        AutoRoutine result = select();
        /**
        LEFT_EMPTY_THEN_BACK(0),
        LEFT_EMPTY_THEN_FRONT(1),
        
        CENTER_EMPTY_THEN_BACK(2),
        CENTER_EMPTY_THEN_FRONT(3),

        RIGHT_EMPTY_THEN_BACK(4),
        RIGHT_EMPTY_THEN_FRONT(5),

        RIGHT_TO_TRENCH_SHOT(6),
         */
        // switch(result) {
        //     case LEFT_EMPTY_THEN_BACK:
        //       return new ShootFromAngleThenTimeDrive(leftStart, -166, getWaitTime(), -0.3, driveBase, intake, conveyor, shooter, indexer, turret);
        //     case LEFT_EMPTY_THEN_FRONT:
        //       return new ShootFromAngleThenTimeDrive(leftStart, -166, getWaitTime(), 0.5, driveBase, intake, conveyor, shooter, indexer, turret);
        //     case CENTER_EMPTY_THEN_BACK:
        //       return new ShootFromAngleThenTimeDrive(centerStart, 0, getWaitTime(), 0.3, driveBase, intake, conveyor, shooter, indexer, turret);
        //     case CENTER_EMPTY_THEN_FRONT:
        //       return new ShootFromAngleThenTimeDrive(centerStart, 0, getWaitTime(), -0.5, driveBase, intake, conveyor, shooter, indexer, turret);
        //     case RIGHT_EMPTY_THEN_BACK:
        //       return new ShootFromAngleThenTimeDrive(rightStart, -27, getWaitTime(), 0.3, driveBase, intake, conveyor, shooter, indexer, turret);
        //     case RIGHT_EMPTY_THEN_FRONT:
        //       return new ShootFromAngleThenTimeDrive(rightStart, -27, getWaitTime(), -0.5, driveBase, intake, conveyor, shooter, indexer, turret);
        //     case RIGHT_TO_TRENCH_SHOT:
        //       return new ToTrenchRunAndShoot(-27, driveBase, intake, conveyor, indexer, turret, shooter);
        //     default:
        //       return new ShootFromAngleThenTimeDrive(centerStart, 0, 0, 0.3, driveBase, intake, conveyor, shooter, indexer, turret);
        //   }

          switch(result) {

            case RightTrenchLoop3Line_GODSPEED:
              return new TrenchLoop3Line(StartPosition.RIGHT, driveBase, intake, conveyor, indexer, turret, shooter);
            case RightTrenchLoop2Line:
              return new TrenchLoop2Line(StartPosition.RIGHT, driveBase, intake, conveyor, indexer, turret, shooter);
            case CenterTrenchLoop3Line_GODSPEED:
              return new TrenchLoop3Line(StartPosition.CENTER, driveBase, intake, conveyor, indexer, turret, shooter);
            case CenterTrenchLoop2Line_GODSPEED:
              return new TrenchLoop2Line(StartPosition.CENTER, driveBase, intake, conveyor, indexer, turret, shooter);

            case Center3Line:
              return new ShootThen3Line(StartPosition.CENTER, driveBase, intake, conveyor, indexer, turret, shooter);
            case Center2Line:
              return new ShootThen2Line(StartPosition.CENTER, driveBase, intake, conveyor, indexer, turret, shooter);
            case Left3Line:
              return new ShootThen3Line(StartPosition.LEFT, driveBase, intake, conveyor, indexer, turret, shooter);
            case Left2Line:
              return new ShootThen2Line(StartPosition.LEFT, driveBase, intake, conveyor, indexer, turret, shooter);

            case CenterDiagonal:
              return new ShootThen5Diagonal(StartPosition.CENTER, driveBase, intake, conveyor, indexer, turret, shooter);
            case RightDiagonal:
              return new ShootThen5Diagonal(StartPosition.RIGHT, driveBase, intake, conveyor, indexer, turret, shooter);
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
            default:
              return new ShootThenForward(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor);
          }
        // return 
        //     new SelectCommand(          
        //         Map.ofEntries(
                    // entry(AutoRoutine.LEFT_EMPTY_THEN_BACK, new ShootFromAngleThenTimeDrive(leftStart, -166, getWaitTime(), -0.3, driveBase, intake, conveyor, shooter, indexer, turret)),
                    // entry(AutoRoutine.LEFT_EMPTY_THEN_FRONT, new ShootFromAngleThenTimeDrive(leftStart, -166, getWaitTime(), 0.5, driveBase, intake, conveyor, shooter, indexer, turret)),
                    
                    // entry(AutoRoutine.CENTER_EMPTY_THEN_BACK, new ShootFromAngleThenTimeDrive(centerStart, 0, getWaitTime(), 0.3, driveBase, intake, conveyor, shooter, indexer, turret)),
                    // entry(AutoRoutine.CENTER_EMPTY_THEN_FRONT, new ShootFromAngleThenTimeDrive(centerStart, 0, getWaitTime(), -0.5, driveBase, intake, conveyor, shooter, indexer, turret)),
                    
                    // entry(AutoRoutine.RIGHT_EMPTY_THEN_BACK, new ShootFromAngleThenTimeDrive(rightStart, -27, getWaitTime(), 0.3, driveBase, intake, conveyor, shooter, indexer, turret)),
                    // entry(AutoRoutine.RIGHT_EMPTY_THEN_FRONT, new ShootFromAngleThenTimeDrive(rightStart, -27, getWaitTime(), -0.5, driveBase, intake, conveyor, shooter, indexer, turret)),
                    // entry(AutoRoutine.RIGHT_TO_TRENCH_SHOT, new ToTrenchRunAndShoot(-27, driveBase, intake, conveyor, indexer, turret, shooter))
                    // entry(AutoRoutine.LEFT_TO_GENERATOR_2_BALL_SIDE, new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.LEFT_TO_GENERATOR_3_BALL_SIDE, new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.LEFT_TO_TRENCH, new ShootFromBaseLineThenToTrench(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.LEFT_TO_GENERATOR_2_TO_TRENCH, new SequentialCommandGroup(
                    //     new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                    //     new Generator2BallSideToTrenchThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    // )),
                    // entry(AutoRoutine.LEFT_TO_GENERATOR_3_TO_2_BALL_SIDE, new SequentialCommandGroup(
                    //     new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                    //     new Generator3BallMidToGenerator2BallMidThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    // )),

                    // entry(AutoRoutine.CENTER_TO_GENERATOR_2_BALL_SIDE, new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.CENTER_TO_GENERATOR_3_BALL_SIDE, new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.CENTER_TO_TRENCH, new ShootFromBaseLineThenToTrench(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.CENTER_TO_GENERATOR_2_TO_TRENCH, new SequentialCommandGroup(
                    //     new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                    //     new Generator2BallSideToTrenchThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    // )),
                    // entry(AutoRoutine.CENTER_TO_GENERATOR_3_TO_2_BALL_SIDE, new SequentialCommandGroup(
                    //     new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                    //     new Generator3BallMidToGenerator2BallMidThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    // )),

                    // entry(AutoRoutine.RIGHT_TO_GENERATOR_2_BALL_SIDE, new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.RIGHT_TO_GENERATOR_3_BALL_SIDE, new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.RIGHT_TO_TRENCH, new ShootFromBaseLineThenToTrench(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    // entry(AutoRoutine.RIGHT_TO_GENERATOR_2_TO_TRENCH, new SequentialCommandGroup(
                    //     new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                    //     new Generator2BallSideToTrenchThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    // )),
                    // entry(AutoRoutine.RIGHT_TO_GENERATOR_3_TO_2_BALL_SIDE, new SequentialCommandGroup(
                    //     new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                    //     new Generator3BallMidToGenerator2BallMidThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    // ))                    
        // ),
        // this::select
        // );
    }

}