package frc.team670.robot.commands.auton;

import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator2BallSide;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToGenerator3BallMid;
import frc.team670.robot.commands.auton.baseline.ShootFromBaseLineThenToTrench;
import frc.team670.robot.commands.auton.generator.Generator2BallSideToTrenchThenShoot;
import frc.team670.robot.commands.auton.generator.Generator3BallMidToGenerator2BallMidThenShoot;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;

public class AutoSelector {

    private DriveBase driveBase;
    private Intake intake;
    private Conveyor conveyor;
    private Indexer indexer;
    private Shooter shooter;
    private Turret turret;
    private MustangCoprocessor coprocessor;

    public AutoSelector(DriveBase driveBase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Turret turret, MustangCoprocessor coprocessor){
        this.driveBase = driveBase;
        this.intake = intake;
        this.conveyor = conveyor;
        this.indexer = indexer;
        this.shooter = shooter;
        this.turret = turret;
        this.coprocessor = coprocessor;
    }

    public static enum AutoRoutine {

        LEFT_TO_GENERATOR_2_BALL_SIDE(0), 
        LEFT_TO_GENERATOR_3_BALL_SIDE(1), 
        LEFT_TO_TRENCH(2),
        LEFT_TO_GENERATOR_2_TO_TRENCH(3), 
        LEFT_TO_GENERATOR_3_TO_2_BALL_SIDE(4),

        CENTER_TO_GENERATOR_2_BALL_SIDE(5), 
        CENTER_TO_GENERATOR_3_BALL_SIDE(6), 
        CENTER_TO_TRENCH(7),
        CENTER_TO_GENERATOR_2_TO_TRENCH(8), 
        CENTER_TO_GENERATOR_3_TO_2_BALL_SIDE(9),

        RIGHT_TO_GENERATOR_2_BALL_SIDE(10), 
        RIGHT_TO_GENERATOR_3_BALL_SIDE(11), 
        RIGHT_TO_TRENCH(12),
        RIGHT_TO_GENERATOR_2_TO_TRENCH(13), 
        RIGHT_TO_GENERATOR_3_TO_2_BALL_SIDE(14),

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
        LEFT, CENTER, RIGHT;
    }

    /**
     * Gets the value of the enum for auto routines based on an int input from the driver dashboard.
     */
    public AutoRoutine select(){
        // TODO: This should be getting the value from driver dashboard
        return AutoRoutine.LEFT_TO_GENERATOR_2_BALL_SIDE;
    }

    /**
     * 
     * @return the command corresponding to the autonomous routine selected by the driver
     */
    public Command getSelectedRoutine(){
        // TODO: based on what value we get from the driver dashboard, returns the command for the appropriate auto routine
        return 
            new SelectCommand(          
                Map.ofEntries(
                    entry(AutoRoutine.LEFT_TO_GENERATOR_2_BALL_SIDE, new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.LEFT_TO_GENERATOR_3_BALL_SIDE, new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.LEFT_TO_TRENCH, new ShootFromBaseLineThenToTrench(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.LEFT_TO_GENERATOR_2_TO_TRENCH, new SequentialCommandGroup(
                        new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                        new Generator2BallSideToTrenchThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    )),
                    entry(AutoRoutine.LEFT_TO_GENERATOR_3_TO_2_BALL_SIDE, new SequentialCommandGroup(
                        new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.LEFT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                        new Generator3BallMidToGenerator2BallMidThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    )),

                    entry(AutoRoutine.CENTER_TO_GENERATOR_2_BALL_SIDE, new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.CENTER_TO_GENERATOR_3_BALL_SIDE, new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.CENTER_TO_TRENCH, new ShootFromBaseLineThenToTrench(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.CENTER_TO_GENERATOR_2_TO_TRENCH, new SequentialCommandGroup(
                        new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                        new Generator2BallSideToTrenchThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    )),
                    entry(AutoRoutine.CENTER_TO_GENERATOR_3_TO_2_BALL_SIDE, new SequentialCommandGroup(
                        new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.CENTER, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                        new Generator3BallMidToGenerator2BallMidThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    )),

                    entry(AutoRoutine.RIGHT_TO_GENERATOR_2_BALL_SIDE, new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.RIGHT_TO_GENERATOR_3_BALL_SIDE, new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.RIGHT_TO_TRENCH, new ShootFromBaseLineThenToTrench(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)),
                    entry(AutoRoutine.RIGHT_TO_GENERATOR_2_TO_TRENCH, new SequentialCommandGroup(
                        new ShootFromBaseLineThenToGenerator2BallSide(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                        new Generator2BallSideToTrenchThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    )),
                    entry(AutoRoutine.RIGHT_TO_GENERATOR_3_TO_2_BALL_SIDE, new SequentialCommandGroup(
                        new ShootFromBaseLineThenToGenerator3BallMid(StartPosition.RIGHT, driveBase, intake, conveyor, shooter, indexer, turret, coprocessor),
                        new Generator3BallMidToGenerator2BallMidThenShoot(driveBase, intake, conveyor, shooter, indexer, turret, coprocessor)
                    ))                    
        ),
        this::select
        );
    }

}