package frc.team670.robot.commands.auton;

import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator2BallSide;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator3BallMid;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToTrench;
import frc.team670.robot.commands.auton.generator.Generator2BallSideToTrenchThenShoot;
import frc.team670.robot.commands.auton.generator.Generator3BallMidToGenerator2BallMidThenShoot;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
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
    private MustangCoprocessor coprocessor;
    
    AutoRoutine selectedRoutine = AutoRoutine.UNKNOWN;

    public AutoSelector(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Turret turret, MustangCoprocessor coprocessor){
        
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");
        
        this.driveBase = driveBase;
        this.intake = intake;
        this.conveyor = conveyor;
        this.indexer = indexer;
        this.shooter = shooter;
        this.turret = turret;
        this.coprocessor = coprocessor;
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
        LEFT_EMPTY_THEN_BACK(0),
        LEFT_EMPTY_THEN_FRONT(1),
        
        CENTER_EMPTY_THEN_BACK(2),
        CENTER_EMPTY_THEN_FRONT(3),

        RIGHT_EMPTY_THEN_BACK(4),
        RIGHT_EMPTY_THEN_FRONT(5),

        RIGHT_TO_TRENCH_SHOT(6),

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

    private Pose2d leftStart = new Pose2d(
        FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS + 
        (FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS - FieldConstants.EDGE_OF_BASELINE), 
        FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(0));
    private Pose2d centerStart = new Pose2d(FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS, 
                   FieldConstants.EDGE_OF_BASELINE, Rotation2d.fromDegrees(180));
    private Pose2d rightStart = new Pose2d(FieldConstants.TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS, 
                   FieldConstants.EDGE_OF_BASELINE,
                   Rotation2d.fromDegrees(180));

    /**
     * Gets the value of the enum for auto routines based on an int input from the
     * driver dashboard.
     * 
     * @return
     */
    public AutoRoutine select() {
        table.addEntryListener("auton-chooser", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double autoID = value.getDouble();
            this.selectedRoutine = AutoRoutine.getById((int)(autoID));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
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
        return 
            (MustangCommand)(new SelectCommand(          
                Map.ofEntries(
                    entry(AutoRoutine.LEFT_EMPTY_THEN_BACK, new ShootFromAngleThenTimeDrive(leftStart, -166, getWaitTime(), -0.3, driveBase, intake, conveyor, shooter, indexer, turret)),
                    entry(AutoRoutine.LEFT_EMPTY_THEN_FRONT, new ShootFromAngleThenTimeDrive(leftStart, -166, getWaitTime(), 0.5, driveBase, intake, conveyor, shooter, indexer, turret)),
                    
                    entry(AutoRoutine.CENTER_EMPTY_THEN_BACK, new ShootFromAngleThenTimeDrive(centerStart, 0, getWaitTime(), 0.3, driveBase, intake, conveyor, shooter, indexer, turret)),
                    entry(AutoRoutine.CENTER_EMPTY_THEN_FRONT, new ShootFromAngleThenTimeDrive(centerStart, 0, getWaitTime(), -0.5, driveBase, intake, conveyor, shooter, indexer, turret)),
                    
                    entry(AutoRoutine.RIGHT_EMPTY_THEN_BACK, new ShootFromAngleThenTimeDrive(rightStart, -27, getWaitTime(), 0.3, driveBase, intake, conveyor, shooter, indexer, turret)),
                    entry(AutoRoutine.RIGHT_EMPTY_THEN_FRONT, new ShootFromAngleThenTimeDrive(rightStart, -27, getWaitTime(), -0.5, driveBase, intake, conveyor, shooter, indexer, turret)),
                    entry(AutoRoutine.RIGHT_TO_TRENCH_SHOT, new ToTrenchRunAndShoot(-27, driveBase, intake, conveyor, indexer, turret, shooter))
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
        ),
        this::select
        ));
    }

}