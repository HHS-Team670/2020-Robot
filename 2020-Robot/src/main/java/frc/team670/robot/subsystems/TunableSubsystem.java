package frc.team670.robot.subsystems;

/**
 * Base for any tunable subsystem
 */
public interface TunableSubsystem {

    public boolean getTimeout();

    /**
     * Enables percent output to shut off other movement of the Subsystem (note this means it will stop holding itself up).
     */
    public void stop();

    /**
     * Moves the Subsystem using percent output
     */
    public void moveByPercentOutput(double output);

}