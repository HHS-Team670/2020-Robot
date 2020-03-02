package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

public class SetRPMTarget extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private double target;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public SetRPMTarget(double rpm, Shooter shooter) {
        this.target = rpm;
        this.shooter = shooter;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        Logger.consoleLog("Setting shooter RPM to %s", target);
        shooter.setVelocityTarget(target);
    }

    @Override
    public void execute() {
        shooter.run();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public void end(boolean interrupt) {
        Logger.consoleLog("Shooter RPM was set, current RPM is %s", shooter.getStage2Velocity());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}