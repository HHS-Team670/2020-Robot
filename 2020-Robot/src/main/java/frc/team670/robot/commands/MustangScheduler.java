package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

import java.util.List;
import java.util.Map;

/**
 * Responsible for scheduling and running commands, including
 * MustangCommandBases. Use this instead of the WPILib CommandScheduler.
 * 
 * @author ctychen, lakshbhambhani
 */

public class MustangScheduler {

    private Command currentCommand;

    private CommandScheduler scheduler;

    private static MustangScheduler instance;

    /**
     * Returns the MustangScheduler instance.
     *
     * @return the instance
     */
    public static synchronized MustangScheduler getInstance() {
        if (instance == null) {
            instance = new MustangScheduler();
        }
        return instance;
    }

    MustangScheduler() {
        scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command -> check(command));
    }

    public void run() {
        scheduler.run();
    }

    public void cancel(MustangCommand... commands) {
        scheduler.cancel((CommandBase[]) commands);
    }

    public Command getCurrentlyScheduled() {
        return this.currentCommand;
    }

    public void cancelAll() {
        scheduler.cancelAll();
    }

    public void schedule(MustangCommand... commands) {

        for (MustangCommand a_command : commands) {

            CommandBase m_command = (CommandBase) a_command;
            try {
                Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> requirements = ((MustangCommand) (m_command))
                        .getHealthRequirements();

                if (requirements != null) {
                    for (MustangSubsystemBase s : requirements.keySet()) {
                        MustangSubsystemBase.HealthState healthReq = requirements.get(s);
                        if (s != null && healthReq != null) {
                            if (s.getHealth(false).getId() > healthReq.getId()) {
                                DriverStation.reportError(
                                        m_command.getName() + " not run because of health issue! Required health: "
                                                + healthReq + ", Actual health: " + s.getHealth(false),
                                        false);
                                Logger.consoleLog(
                                        "%s not run because of health issue! Required health: %s , Actual health: %s",
                                        m_command.getName(), healthReq, s.getHealth(false));
                                return;
                            }
                        }
                    }
                }
                this.currentCommand = m_command;
                scheduler.schedule(currentCommand);
                Logger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
            } finally {
                this.currentCommand = null;
            }
        }
    }

    /**
     * Initial check that runs when the command initializes to check if it is a
     * MustangCommand that has been scheduled using MustangScheduler
     * 
     * @param Command command The command which has been scheduled
     */
    public void check(Command command) throws RuntimeException {
        if (command == null) {
            Logger.consoleLog("Command is null");
            return;
        } else {
            if (!(command instanceof MustangCommand)) {
                throw new RuntimeException("Command was not properly scheduled. Are you using MustangScheduler?");
            }
        }
    }

    public void setDefaultCommand(MustangSubsystemBase subsystem, MustangCommand command) {
        CommandBase m_command = (CommandBase) command;
        try {
            Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> requirements = ((MustangCommand) (m_command))
                    .getHealthRequirements();

            if (requirements != null) {
                for (MustangSubsystemBase s : requirements.keySet()) {
                    MustangSubsystemBase.HealthState healthReq = requirements.get(s);
                    if (s != null && healthReq != null) {
                        if (s.getHealth(false).getId() > healthReq.getId()) {
                            DriverStation.reportError(
                                    m_command.getName() + " not run because of health issue! Required health: "
                                            + healthReq + ", Actual health: " + s.getHealth(false),
                                    false);
                            Logger.consoleLog(
                                    "%s not run because of health issue! Required health: %s , Actual health: %s",
                                    m_command.getName(), healthReq, s.getHealth(false));
                            return;
                        }
                    }
                }
            }
            this.currentCommand = m_command;
            scheduler.setDefaultCommand(subsystem, currentCommand);
            Logger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
        } finally {
            this.currentCommand = null;
        }
    }

}