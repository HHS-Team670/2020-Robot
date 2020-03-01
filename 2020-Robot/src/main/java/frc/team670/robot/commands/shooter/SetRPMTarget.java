package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class SetRPMTarget extends InstantCommand implements MustangCommand {

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
        shooter.setVelocityTarget(target);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}