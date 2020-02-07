package frc.team670.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.robot.utils.Logger;

public class MustangWarnings {

    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("/SmartDashboard");
    private static NetworkTableEntry warning = table.getEntry("warnings");

    public static boolean atCompetition() {
        return DriverStation.getInstance().isFMSAttached();
    }

    public static void reportWarning(String message, Object... parameters) {
        DriverStation.reportWarning(String.format(message, parameters), false);
        Logger.consoleWarning(message, parameters);
        warning.forceSetString(String.format(message, parameters));
        if (!atCompetition())
            throw new RuntimeException(message);
    }

    public static void reportError(String message, Object... parameters) {
        DriverStation.reportError(String.format(message, parameters), false);
        Logger.consoleError(message, parameters);
        warning.forceSetString(String.format(message, parameters));
        if (!atCompetition())
            throw new RuntimeException(message);
    }

    public static void notify(String message, Object... parameters) {
        DriverStation.reportWarning(String.format(message, parameters), false);
        Logger.consoleLog(message, parameters);
        warning.forceSetString(String.format(message, parameters));
    }
}