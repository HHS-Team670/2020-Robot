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
import frc.team670.robot.commands.MustangScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.CancelAllCommands;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.MustangNotifications;

/**
 * Listens on network tables to keys sent over by the XKeys keyboard and calls
 * the corresponding commands
 * 
 * @author lakshbhambhani
 */
public class XKeys {

    private NetworkTableInstance instance;
    private NetworkTable table;

    public XKeys() {
        SmartDashboard.putString("XKEYS", "XKeys constructor");
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");

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
            else if (s == xkeysCommands.BRING_INTAKE_IN)
                bringIntakeIn();
            else if (s == xkeysCommands.BRING_INTAKE_OUT)
                bringIntakeOut();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-shooter", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.INIT_SHOOTER)
                initShooter();
            else if (s == xkeysCommands.SHOOT_HIGH)
                shootHigh();
            else if (s == xkeysCommands.SHOOT_LOW)
                shootLow();
            else if (s == xkeysCommands.SHOOT_ALL)
                shootAll();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-updraw", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.TOGGLE_UPDRAW_UP)
                toggleUpdrawUp();
            else if (s == xkeysCommands.TOGGLE_UPDRAW_DOWN)
                toggleUpdrawDown();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-indexer", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.INDEXER_INTAKE)
                indexerAtIntake();
            else if (s == xkeysCommands.INDEXER_SHOOT)
                indexerAtShoot();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.EXTEND_CLIMBER)
                extendClimber();
            else if (s == xkeysCommands.RETRACT_CLIMBER)
                retractClimber();
            else if (s == xkeysCommands.RETRACT_CLIMBER_LEFT)
                retractClimberLeft();
            else if (s == xkeysCommands.RETRACT_CLIMBER_RIGHT)
                retractClimberRight();
            else if (s == xkeysCommands.EXTEND_CLIMBER_LEFT)
                extendClimberLeft();
            else if (s == xkeysCommands.EXTEND_CLIMBER_RIGHT)
                extendClimberRight();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.CANCEL_ALL)
                cancelAllCommands();
            else if (s == xkeysCommands.CANCEL_INTAKE)
                cancelIntake();
            else if (s == xkeysCommands.CANCEL_INDEXER)
                cancelIndexer();
            else if (s == xkeysCommands.CANCEL_UPDRAW)
                cancelUpdraw();
            else if (s == xkeysCommands.CANCEL_SHOOTER)
                cancenShooter();
            else if (s == xkeysCommands.CANCEL_CLIMBER)
                cancelClimber();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-autopickup", (table2, key2, entry, value, flags) -> {
            autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-vision", (table2, key2, entry, value, flags) -> {
            visionAlign();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    private class xkeysCommands { // do not use enums as getID has to be called over enum call

        public static final double RUN_INTAKE_IN = 0;
        public static final double RUN_INTAKE_OUT = 1;
        public static final double BRING_INTAKE_IN = 2;
        public static final double BRING_INTAKE_OUT = 3;
        public static final double INIT_SHOOTER = 4;
        public static final double SHOOT_HIGH = 5;
        public static final double SHOOT_LOW = 6;
        public static final double SHOOT_ALL = 7;
        public static final double TOGGLE_UPDRAW_UP = 8;
        public static final double TOGGLE_UPDRAW_DOWN = 9;
        public static final double INDEXER_INTAKE = 10;
        public static final double INDEXER_SHOOT = 11;
        public static final double EXTEND_CLIMBER = 12;
        public static final double RETRACT_CLIMBER = 13;
        public static final double EXTEND_CLIMBER_LEFT = 14;
        public static final double EXTEND_CLIMBER_RIGHT = 15;
        public static final double RETRACT_CLIMBER_LEFT = 16;
        public static final double RETRACT_CLIMBER_RIGHT = 17;
        public static final double CANCEL_ALL = 18;
        public static final double CANCEL_INTAKE = 19;
        public static final double CANCEL_INDEXER = 20;
        public static final double CANCEL_UPDRAW = 21;
        public static final double CANCEL_SHOOTER = 22;
        public static final double CANCEL_CLIMBER = 23;
    }

    private void extendClimber() {
        MustangScheduler.getInstance().schedule();
    }

    private void retractClimber() {
        MustangScheduler.getInstance().schedule();
    }

    private void retractClimberLeft() {
        MustangScheduler.getInstance().schedule();
    }

    private void retractClimberRight() {
        MustangScheduler.getInstance().schedule();
    }

    private void extendClimberLeft() {
        MustangScheduler.getInstance().schedule();
    }

    private void extendClimberRight() {
        MustangScheduler.getInstance().schedule();
    }

    private void initShooter() {
        MustangScheduler.getInstance().schedule();
    }

    private void shootHigh() {
        MustangScheduler.getInstance().schedule();
    }

    private void shootLow() {
        MustangScheduler.getInstance().schedule();
    }

    private void shootAll() {
        MustangScheduler.getInstance().schedule();
    }

    private void bringIntakeIn() {
        MustangScheduler.getInstance().schedule();
    }

    private void bringIntakeOut() {
        MustangScheduler.getInstance().schedule();
    }

    private void runIntakeIn() {
        MustangScheduler.getInstance().schedule();
    }

    private void runIntakeOut() {
        MustangScheduler.getInstance().schedule();
    }

    private void autoPickupBall() {
        MustangScheduler.getInstance().schedule();
    }

    private void visionAlign() {
        MustangScheduler.getInstance().schedule();
    }

    private void toggleUpdrawUp() {
        MustangScheduler.getInstance().schedule();
    }

    private void toggleUpdrawDown() {
        MustangScheduler.getInstance().schedule();
    }

    private void indexerAtIntake() {
        MustangScheduler.getInstance().schedule();
    }

    private void indexerAtShoot() {
        MustangScheduler.getInstance().schedule();
    }

    private void cancelAllCommands() {
        MustangScheduler.getInstance().schedule(new CancelAllCommands());
    }

    private void cancelIntake() {
        MustangScheduler.getInstance().schedule();
    }

    private void cancelIndexer() {
        MustangScheduler.getInstance().schedule();
    }

    private void cancelUpdraw() {
        MustangScheduler.getInstance().schedule();
    }

    private void cancenShooter() {
        MustangScheduler.getInstance().schedule();
    }

    private void cancelClimber() {
        MustangScheduler.getInstance().schedule();
    }

}