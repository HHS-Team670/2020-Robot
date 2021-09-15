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

        public static final double RUN_INTAKE_IN = 0;
        public static final double RUN_INTAKE_OUT = 1;
        public static final double TOGGLE_INTAKE = 2;
        
        public static final double INIT_SHOOTER = 4;
        public static final double SHOOT = 6;
        public static final double SHOOT_ALL = 7;

        public static final double INCREASE_SHOOTER_RPM = 8;
        public static final double DECREASE_SHOOTER_RPM = 9;

        // public static final double INDEXER_INTAKE = 10;

        public static final double EXTEND_CLIMBER = 12;
        public static final double RETRACT_CLIMBER = 13;

        public static final double SHOOT_NEAR = 14;
        public static final double SHOOT_MID = 15;
        public static final double SHOOT_LONG = 16;

        public static final double CANCEL_ALL = 18;
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
            else if (s == xkeysCommands.TOGGLE_INTAKE)
                toggleIntake();
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
            else if (s == xkeysCommands.INCREASE_SHOOTER_RPM)
                increaseShooterSpeed();
            else if (s == xkeysCommands.DECREASE_SHOOTER_RPM)
                decreaseShooterSpeed();
            else if (s == xkeysCommands.SHOOT_NEAR)
                setCloseShotSpeed();
            else if (s == xkeysCommands.SHOOT_MID) 
                setMidShotSpeed();
            else if (s == xkeysCommands.SHOOT_LONG)
                setLongShotSpeed();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-indexer", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            //if (s == xkeysCommands.INDEXER_INTAKE)
                // indexerAtIntake();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.EXTEND_CLIMBER)
                extendClimber();
            else if (s == xkeysCommands.RETRACT_CLIMBER)
                retractClimber();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.CANCEL_ALL)
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

    private void retractClimber() {
        MustangScheduler.getInstance().schedule(new Climb(climber));
    }

    private void initShooter() {
        MustangScheduler.getInstance().schedule(new StartShooterByDistance(shooter, drivebase));
    }

    private void shoot() {
        MustangScheduler.getInstance().schedule(new Shoot(shooter));
    }

    private void increaseShooterSpeed() {
        shooter.adjustRPMAdjuster(50.0);
    }

    private void decreaseShooterSpeed() {
        shooter.adjustRPMAdjuster(-50.0);
    }

    private void setCloseShotSpeed() {
        shooter.setVelocityTarget(2125);
    }

    private void setMidShotSpeed() {
        shooter.setVelocityTarget(2275);
    }

    private void setLongShotSpeed() {
        shooter.setVelocityTarget(2725);
    }

    private void shootAll() {
        // MustangScheduler.getInstance().schedule(new ShootAllBalls(indexer));
    }

    private void toggleIntake() {
        MustangScheduler.getInstance().schedule(new DeployIntake(!intake.isDeployed(), intake));
    }

    private void runIntakeIn() {
        MustangScheduler.getInstance().schedule(new RunIntake(false, intake));
    }

    private void runIntakeOut() {
        MustangScheduler.getInstance().schedule(new RunIntake(true, intake));
    }

    private void autoPickupBall() {
        // MustangScheduler.getInstance().schedule(new ShootAllBalls(indexer));
    }

    private void visionAlign() {
        MustangScheduler.getInstance().schedule(new GetVisionData(coprocessor, drivebase));
    }

    // private void indexerAtIntake() {
    //     MustangScheduler.getInstance().schedule(new RotateToIntakePosition(indexer));
    // }

    private void cancelAllCommands() {
        MustangScheduler.getInstance().schedule(new CancelAllCommands());
    }
}