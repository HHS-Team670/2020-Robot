/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.robot.commands.CancelAllCommands;
import frc.team670.robot.commands.climb.Climb;
import frc.team670.robot.commands.climb.ExtendClimber;
// import frc.team670.robot.commands.indexer.s;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.shooter.Shoot;
// import frc.team670.robot.commands.routines.IntakeBallToIndexer;
// import frc.team670.robot.commands.routines.RotateIndexerToUptakeThenShoot;
import frc.team670.robot.commands.shooter.StartShooterByDistance;
import frc.team670.robot.commands.vision.GetVisionData;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;

/**
 * Listens on network tables to keys sent over by the XKeys keyboard and calls
 * the corresponding commands
 * 
 * @author lakshbhambhani
 */
public class XKeys {

    private static NetworkTableInstance instance;
    private static NetworkTable table, visionTable;

    private DriveBase drivebase;
    private Climber climber;
    private Intake intake;
    private Shooter shooter;
    private Conveyor conveyor;
    private Indexer indexer;
    private Turret turret;
    private Vision coprocessor;
    private class xkeysCommands { // do not use enums as getID has to be called over enum call

        public static final double EXTEND_CLIMBER = 1;
        public static final double DRIVE_TO_BAR_AND_PREPARE_CLIMB = 2;
        public static final double RETRACT_CLIMBER = 3;
        public static final double HOOK_ON_BAR = 4;

        public static final double INIT_SHOOTER = 5;
        public static final double SHOOT = 6;
        public static final double START_SHOOTER = 7;
        public static final double VISION_SHOOTER = 8;
        public static final double INCREASE_SHOOTER_SPEED = 9;
        public static final double DECREASE_SHOOTER_SPEED = 10;
        public static final double SET_CLOSE_SHOT_SPEED = 11;
        public static final double SET_MID_SHOT_SPEED = 12;
        public static final double SET_LONG_SHOT_SPEED = 13;
        public static final double SHOOT_ALL = 14;

        public static final double TOGGLE_INTAKE = 15;
        public static final double DEPLOY_INTAKE = 16;
        public static final double RETRACT_INTAKE = 17;
        public static final double RUN_INTAKE_IN = 18;
        public static final double RUN_INTAKE_CONVEYOR_IN = 19;
        public static final double RUN_INTAKE_CONVEYOR_OUT = 20;
        public static final double RUN_INTAKE_OUT = 21;
        public static final double RUN_CONVEYOR_OUT = 22;
        public static final double RUN_CONVEYOR_IN = 23;
        public static final double STOP_INTAKE = 24;
        public static final double AUTO_PICKUP_BALL = 25;

        public static final double VISION_ALIGN = 26;
        public static final double AUTO_ROTATE = 27;
        public static final double ALIGN_TURRET = 28;
        public static final double ROTATE_TURRET_L = 29;
        public static final double ROTATE_TURRET_R = 30;
        public static final double ROTATE_TURRET_TO_HOME = 31;
        public static final double TURN_TURRET = 32;
        public static final double ZERO_TURRET = 33;

        public static final double MANUAL_INDEXER = 34;
        public static final double MANUAL_INDEXER_REV = 35;
        public static final double INDEXER = 36;
        public static final double UPDRAW = 37;
        public static final double CANCEL_ALL_COMMANDS = 38;
    }

    /**
     * Used to create an xkeys object
     * @param drivebase the drivebase to control
     * @param intake the intake to run/control
     * @param conveyor the conveyor to run
     * @param indexer the indexer to run/control
     * @param shooter the shooter to run/control
     * @param climber the climber to run
     * @param turret the turret to run/control
     * @param vision the coprocessor specfically the pi
     */
    public XKeys(DriveBase drivebase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Climber climber, Turret turret, Vision vision) {
        SmartDashboard.putString("XKEYS", "XKeys constructor");
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");
        visionTable = instance.getTable("Vision");

        this.drivebase = drivebase;
        this.intake = intake;
        this.conveyor = conveyor;
        this.indexer = indexer;
        this.shooter = shooter;
        this.climber = climber;
        this.turret = turret;
        this.coprocessor = vision;

        table.addEntryListener((table2, key2, entry, value, flags) -> {
            try {
                Logger.consoleLog("key pressed: " + value.getString());
            } catch (Exception e) {
                MustangNotifications.reportMinorWarning("Could not detect what key was pressed");
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-intake", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.RUN_INTAKE_IN)
                runIntakeIn();
            else if (s == xkeysCommands.RUN_INTAKE_OUT)
                runIntakeOut();
            else if (s == xkeysCommands.RUN_INTAKE_CONVEYOR_IN)
                runIntakeConveyorIn();
            else if (s == xkeysCommands.RUN_INTAKE_CONVEYOR_OUT)
                runIntakeConveyorOut();
            else if (s == xkeysCommands.RUN_CONVEYOR_IN)
                runConveyorIn();
            else if (s == xkeysCommands.RUN_CONVEYOR_OUT)
                runConveyorOut();
            else if (s == xkeysCommands.TOGGLE_INTAKE)
                toggleIntake();
            else if (s == xkeysCommands.DEPLOY_INTAKE)
                deployIntake();
            else if (s == xkeysCommands.RETRACT_INTAKE)
                retractIntake();
            else if (s == xkeysCommands.STOP_INTAKE)
                stopIntake();
            else if (s == xkeysCommands.AUTO_PICKUP_BALL)
                autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-shooter", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.INIT_SHOOTER)
                initShooter();
            else if (s == xkeysCommands.SHOOT)
                shoot();
            else if (s == xkeysCommands.SHOOT_ALL)
                shootAll();
            else if (s == xkeysCommands.INCREASE_SHOOTER_SPEED)
                increaseShooterSpeed();
            else if (s == xkeysCommands.DECREASE_SHOOTER_SPEED)
                decreaseShooterSpeed();
            else if (s == xkeysCommands.SET_CLOSE_SHOT_SPEED)
                setCloseShotSpeed();
            else if (s == xkeysCommands.SET_MID_SHOT_SPEED) 
                setMidShotSpeed();
            else if (s == xkeysCommands.SET_LONG_SHOT_SPEED)
                setLongShotSpeed();
            else if (s == xkeysCommands.START_SHOOTER)
                startShooter();
            else if (s == xkeysCommands.VISION_SHOOTER)
                visionShooter();
            else if (s == xkeysCommands.AUTO_ROTATE)
                autoRotate();
                else if (s == xkeysCommands.VISION_ALIGN)
                visionAlign();
                else if (s == xkeysCommands.ALIGN_TURRET)
                alignTurret();
                else if (s == xkeysCommands.ROTATE_TURRET_L)
                rotateTurretL();
                else if (s == xkeysCommands.ROTATE_TURRET_R)
                rotateTurretR();
                else if (s == xkeysCommands.ROTATE_TURRET_TO_HOME)
                rotateTurretToHome();
                else if (s == xkeysCommands.TURN_TURRET)
                turnTurret();
                else if (s == xkeysCommands.ZERO_TURRET)
                zeroTurret();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-indexer", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.MANUAL_INDEXER)
                manualIndexer();
            else if (s == xkeysCommands.MANUAL_INDEXER_REV)
                manualIndexerRev();
                else if (s == xkeysCommands.INDEXER)
                indexer();
                else if (s == xkeysCommands.UPDRAW)
                updraw();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.EXTEND_CLIMBER)
                extendClimber();
            else if (s == xkeysCommands.RETRACT_CLIMBER)
                retractClimber();
            else if (s == xkeysCommands.DRIVE_TO_BAR_AND_PREPARE_CLIMB)
                driveToBarAndPrepareClimb();
            else if (s == xkeysCommands.HOOK_ON_BAR)
                hookOnBar();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.CANCEL_ALL_COMMANDS)
                cancelAllCommands();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-autopickup", (table2, key2, entry, value, flags) -> {
            autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        visionTable.addEntryListener("vision-data", (table2, key2, entry, value, flags) -> {
            visionAlign();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    private void extendClimber() {
        MustangScheduler.getInstance().schedule(new ExtendClimber(climber));
    }

    pprivate void driveToBarAndPrepareClimb() {
        MustangScheduler.getInstance().schedule(new DriveToBarAndPrepareClimb(drivebase, climber));
    }

    private void retractClimber() {
        MustangScheduler.getInstance().schedule(new Climb(climber));
    }

    private void hookOnBar() {
        MustangScheduler.getInstance().schedule(new HookOnBar(climber));
    }

    private void initShooter() {
        MustangScheduler.getInstance().schedule(new StartShooterByPoseDistance(shooter, drivebase));
    }

    private void shoot() {
        MustangScheduler.getInstance().schedule(new Shoot(shooter));
    }

    private void startShooter() {
        MustangScheduler.getInstance().schedule(new StartShooter(shooter));
    }

    private void visionShooter() {
        MustangScheduler.getInstance().schedule(new StartShooterByVisionDistance(shooter, coprocessor));
    }

    private void increaseShooterSpeed() {
        MustangScheduler.getInstance().schedule(new SetRPMAdjuster(50.0, shooter));
    }

    private void decreaseShooterSpeed() {
        MustangScheduler.getInstance().schedule(new SetRPMAdjuster(-50.0, shooter));
    }

    private void setCloseShotSpeed() {
        MustangScheduler.getInstance().schedule(new SetRPMTarget(2125, shooter));
    }

    private void setMidShotSpeed() {
        MustangScheduler.getInstance().schedule(new SetRPMTarget(2275, shooter));
    }

    private void setLongShotSpeed() {
        MustangScheduler.getInstance().schedule(new SetRPMTarget(2725, shooter));
    }

    private void shootAll() {
        MustangScheduler.getInstance().schedule(new ShootAllBalls(indexer));
    }

    private void toggleIntake() {
        MustangScheduler.getInstance().schedule(new ToggleIntake(intake));
    }

    private void deployIntake() {
        MustangScheduler.getInstance().schedule(new DeployIntake(false, intake));
    }

    private void retractIntake() {
        MustangScheduler.getInstance().schedule(new DeployIntake(true, intake));
    }

    private void runIntakeIn() {
        MustangScheduler.getInstance().schedule(new RunIntake(false, intake));
    }

    private void runIntakeConveyorIn() {
        MustangScheduler.getInstance().schedule(new RunIntakeConveyor(intake, conveyor, indexer, false));
    }

    private void runIntakeConveyorOut() {
        MustangScheduler.getInstance().schedule(new RunIntakeConveyor(intake, conveyor, indexer, true));
    }

    private void runIntakeOut() {
        MustangScheduler.getInstance().schedule(new RunIntake(true, intake));
    }

    private void runConveyorOut() {
        MustangScheduler.getInstance().schedule(new RunConveyor(true, conveyor, indexer));
    }

    private void runConveyorIn() {
        MustangScheduler.getInstance().schedule(new RunConveyor(false, conveyor, indexer));
    }

    private void stopIntake() {
        MustangScheduler.getInstance().schedule(new StopIntake(intake));
    }

    private void autoPickupBall() {
        MustangScheduler.getInstance().schedule(new ShootAllBalls(indexer, conveyor, shooter, coprocessor));
    }

    private void visionAlign() {
        MustangScheduler.getInstance().schedule(new GetVisionData(coprocessor, drivebase));
    }

    private void autoRotate() {
        MustangScheduler.getInstance().schedule(new AutoRotate(turret, coprocessor, drivebase));
    }

    private void alignTurret() {
        MustangScheduler.getInstance().schedule(new GetLatestDataAndAlignTurret(turret, drivebase, coprocessor));
    }

    private void rotateTurretL() {
        MustangScheduler.getInstance().schedule(new RotateToAngle(turret, 20.0));
    }

    private void rotateTurretR() {
        MustangScheduler.getInstance().schedule(new RotateToAngle(turret, -20.0));
    }

    private void rotateTurretToHome() {
        MustangScheduler.getInstance().schedule(new RotateToHome(turret));
    }

    private void turnTurret() {
        MustangScheduler.getInstance().schedule(new RotateTurret(turret, drivebase, coprocessor));
    }

    private void zeroTurret() {
        MustangScheduler.getInstance().schedule(new ZeroTurret(turret));
    }

    private void manualIndexer() {
        MustangScheduler.getInstance().schedule(new ManualRunIndexer(indexer, conveyor, intake, false));
    }

    private void manualIndexerRev() {
        MustangScheduler.getInstance().schedule(new ManualRunIndexer(indexer, conveyor, intake, true));
    }

    private void indexer() {
        MustangScheduler.getInstance().schedule(new Indexer(indexer, conveyor));
    }

    private void updraw() {
        MustangScheduler.getInstance().schedule(new ToggleUpdraw(indexer));
    }

    private void cancelAllCommands() {
        MustangScheduler.getInstance().schedule(new CancelAllCommands());
    }
}