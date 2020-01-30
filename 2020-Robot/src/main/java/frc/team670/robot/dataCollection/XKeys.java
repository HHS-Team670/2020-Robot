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
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-shooter", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("toggle_shooter_high"))
                toggleShooterHigh();
            else if (s.equals("toggle_shooter_low"))
                toggleShooterLow();
            else if (s.equals("toggle_shooter_zero"))
                toggleShooterZero();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("raise_climber"))
                raiseClimber();
            else if (s.equals("lower_climber"))
                lowerClimber();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-autopickup", (table2, key2, entry, value, flags) -> {
            autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-visiondrive", (table2, key2, entry, value, flags) -> {
            visionAlign();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString)
                return;
            String s = value.getString();
            if (s.equals("cancel_all"))
                cancelAllCommands();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    private void raiseClimber(){
        MustangScheduler.getInstance().schedule();
    }

    private void lowerClimber(){
        MustangScheduler.getInstance().schedule();
    }

    private void toggleShooterHigh(){
        MustangScheduler.getInstance().schedule();
    }

    private void toggleShooterLow(){
        MustangScheduler.getInstance().schedule();
    }

    private void toggleShooterZero(){
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

    private void cancelAllCommands() {
        MustangScheduler.getInstance().schedule(new CancelAllCommands());
    }

    private void visionAlign() {
        MustangScheduler.getInstance().schedule();
    }

    private void autoPickupBall() {
        MustangScheduler.getInstance().schedule();
    }

}