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
            if (s == xkeysCommands.run_intake_in)
                runIntakeIn();
            else if (s == xkeysCommands.run_intake_out)
                runIntakeOut();
            else if (s == xkeysCommands.bring_intake_in)
                bringIntakeIn();
            else if (s == xkeysCommands.bring_intake_out)
                bringIntakeOut();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-shooter", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.init_shooter)
                initShooter();
            else if (s == xkeysCommands.shoot_high)
                shootHigh();
            else if (s == xkeysCommands.shoot_low)
                shootLow();
            else if (s == xkeysCommands.shoot_all)
                shootAll();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-updraw", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.toggle_updraw_up)
                toggleUpdrawUp();
            else if (s == xkeysCommands.toggle_updraw_down)
                toggleUpdrawDown();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-indexer", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.indexer_intake)
                indexerAtIntake();
            else if (s == xkeysCommands.indexer_shoot)
                indexerAtShoot();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.extend_climber)
                extendClimber();
            else if (s == xkeysCommands.retract_climber)
                retractClimber();
            else if (s == xkeysCommands.retract_climber_left)
                retractClimberLeft();
            else if (s == xkeysCommands.retract_climber_right)
                retractClimberRight();
            else if (s == xkeysCommands.extend_climber_left)
                extendClimberLeft();
            else if (s == xkeysCommands.extend_climber_right)
                extendClimberRight();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.cancel_all)
                cancelAllCommands();
            else if (s == xkeysCommands.cancel_intake)
                cancelIntake();
            else if (s == xkeysCommands.cancel_indexer)
                cancelIndexer();
            else if (s == xkeysCommands.cancel_updraw)
                cancelUpdraw();
            else if (s == xkeysCommands.cancel_shooter)
                cancenShooter();
            else if (s == xkeysCommands.cancel_climber)
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

        public static final double run_intake_in = 0;
        public static final double run_intake_out = 1;
        public static final double bring_intake_in = 2;
        public static final double bring_intake_out = 3;
        public static final double init_shooter = 4;
        public static final double shoot_high = 5;
        public static final double shoot_low = 6;
        public static final double shoot_all = 7;
        public static final double toggle_updraw_up = 8;
        public static final double toggle_updraw_down = 9;
        public static final double indexer_intake = 10;
        public static final double indexer_shoot = 11;
        public static final double extend_climber = 12;
        public static final double retract_climber = 13;
        public static final double extend_climber_left = 14;
        public static final double extend_climber_right = 15;
        public static final double retract_climber_left = 16;
        public static final double retract_climber_right = 17;
        public static final double cancel_all = 18;
        public static final double cancel_intake = 19;
        public static final double cancel_indexer = 20;
        public static final double cancel_updraw = 21;
        public static final double cancel_shooter = 22;
        public static final double cancel_climber = 23;
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

    private void shoot() {
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