package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.Logger;

import java.util.Map;

/**
 * Responsible for scheduling and running commands, including
 * MustangCommandBases. Use this instead of the WPILib CommandScheduler.
 * 
 * @author ctychen
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

    public void check(Command command) throws RuntimeException {
        if (!this.currentCommand.equals(command)) {
            throw new RuntimeException("Command was not properly scheduled. Are you using MustangScheduler?");
        }
    }

    public void setDefaultCommand(MustangSubsystemBase subsystem, MustangCommand command) {
        CommandScheduler.getInstance().setDefaultCommand((SubsystemBase) subsystem, (CommandBase) command);
    }

}