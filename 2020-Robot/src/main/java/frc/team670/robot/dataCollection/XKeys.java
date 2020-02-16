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
import frc.team670.robot.commands.climber.ExtendClimber;
import frc.team670.robot.commands.climber.RetractClimber;
import frc.team670.robot.commands.indexer.RotateToIntakePosition;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.routines.IntakeBallToIndexer;
import frc.team670.robot.commands.routines.RotateIndexerToUptakeThenShoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.climber.Climber;
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

    private Climber climber;
    private Intake intake;
    private Shooter shooter;
    private Conveyor conveyor;
    private Indexer indexer;

    public XKeys(Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Climber climber) {
        SmartDashboard.putString("XKEYS", "XKeys constructor");
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");

        this.intake = intake;
        this.conveyor = conveyor;
        this.indexer = indexer;
        this.shooter = shooter;
        this.climber = climber;

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
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-indexer", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kDouble)
                return;
            double s = value.getDouble();
            if (s == xkeysCommands.INDEXER_INTAKE)
                indexerAtIntake();
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
        table.addEntryListener("xkeys-vision", (table2, key2, entry, value, flags) -> {
            visionAlign();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    private class xkeysCommands { // do not use enums as getID has to be called over enum call

        public static final double RUN_INTAKE_IN = 0;
        public static final double RUN_INTAKE_OUT = 1;
        public static final double TOGGLE_INTAKE = 2;
        public static final double INIT_SHOOTER = 4;
        public static final double SHOOT = 6;
        public static final double SHOOT_ALL = 7;
        public static final double INDEXER_INTAKE = 10;
        public static final double EXTEND_CLIMBER = 12;
        public static final double RETRACT_CLIMBER = 13;
        public static final double CANCEL_ALL = 18;
    }

    private void extendClimber() {
        MustangScheduler.getInstance().schedule(new ExtendClimber(climber, 198.21));
    }

    private void retractClimber() {
        MustangScheduler.getInstance().schedule(new RetractClimber(climber, 0));
    }

    private void initShooter() {
        MustangScheduler.getInstance().schedule(new StartShooter(shooter));
    }

    private void shoot() {
        MustangScheduler.getInstance().schedule(new RotateIndexerToUptakeThenShoot(indexer, shooter));
    }

    private void shootAll() {
        MustangScheduler.getInstance().schedule();
    }

    private void toggleIntake() {
        MustangScheduler.getInstance().schedule(new DeployIntake(!intake.isDeployed(), intake));
    }

    private void runIntakeIn() {
        MustangScheduler.getInstance().schedule(new RunIntake(-0.7, intake));
    }

    private void runIntakeOut() {
        MustangScheduler.getInstance().schedule(new RunIntake(0.7, intake));
    }

    private void autoPickupBall() {
        MustangScheduler.getInstance().schedule(new IntakeBallToIndexer(intake, conveyor, indexer));
    }

    private void visionAlign() {
        MustangScheduler.getInstance().schedule();
    }

    private void indexerAtIntake() {
        MustangScheduler.getInstance().schedule(new RotateToIntakePosition(indexer));
    }

    private void cancelAllCommands() {
        MustangScheduler.getInstance().schedule(new CancelAllCommands());
    }
}