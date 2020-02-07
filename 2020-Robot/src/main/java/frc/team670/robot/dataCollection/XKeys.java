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

/**
 * Listens on network tables to keys sent over by the XKeys keyboard and calls
 * the corresponding commands
 *
 * Link to XKeys bindings:
 * https://docs.google.com/spreadsheets/d/1Y1cZvWabaVvush9LvfwKdRgCdmRTlztb67nWk5D-5x4/edit?usp=sharing
 * Link to Dashboard where XKeys are read in and values are sent over
 * networktables: https://github.com/HHS-Team670/FRCDashboard
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
                System.out.println("key pressed: " + value.getString());
                Logger.consoleLog("key pressed: " + value.getString());
            } catch (Exception e) {
                Logger.consoleLog("Could not log or detect key press");
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-intake", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("toggle_intake_in"))
                runIntakeIn();
            else if (s.equals("toggle_intake_out"))
                runIntakeOut();
            else if (s.equals("bring_intake_in"))
                bringIntakeIn();
            else if (s.equals("bring_intake_out"))
                bringIntakeOut();
            else if (s.equals("auto_pickup_ball"))
                autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-shooter", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("init_shooter"))
                initShooter();
            else if (s.equals("shoot_high"))
                shootHigh();
            else if (s.equals("shoot_low"))
                shootLow();
            else if (s.equals("shoot"))
                shoot();
            else if (s.equals("shoot_all"))
                shootAll();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-updraw", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("toggle_updraw_up"))
                toggleUpdrawUp();
            else if (s.equals("toggle_updraw_down"))
                toggleUpdrawDown();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-indexer", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("indexer_1"))
                rotateIndexerBy1();
            else if (s.equals("indexer_-1"))
                rotateIndexerBackBy1();
            else if (s.equals("rotate_indexer"))
                rotateIndexer();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("extend_climber"))
                extendClimber();
            else if (s.equals("retract_climber"))
                retractClimber();
            else if (s.equals("retract_climber_left"))
                retractClimberLeft();
            else if (s.equals("retract_climber_right"))
                retractClimberRight();
            else if (s.equals("extend_climber_left"))
                extendClimberLeft();
            else if (s.equals("extend_climber_right"))
                extendClimberRight();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("cancel_all"))
                cancelAllCommands();
            else if (s.equals("cancel_intake"))
                cancelIntake();
            else if (s.equals("cancel_indexer"))
                cancelIndexer();
            else if (s.equals("cancel_updraw"))
                cancelUpdraw();
            else if (s.equals("cancel_shooter"))
                cancenShooter();
            else if (s.equals("cancel_climber"))
                cancelClimber();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-autopickup", (table2, key2, entry, value, flags) -> {
            autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-vision", (table2, key2, entry, value, flags) -> {
            visionAlign();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
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

    private void rotateIndexerBy1() {
        MustangScheduler.getInstance().schedule();
    }

    private void rotateIndexerBackBy1() {
        MustangScheduler.getInstance().schedule();
    }

    private void rotateIndexer() {
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