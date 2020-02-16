package frc.team670.robot.commands.turret;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Rotates the turret to a specified angle
 */
public class RotateTurret extends CommandBase implements MustangCommand {

    private Turret turret;
    private double angle;
    private double ERROR_MARGIN;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * @param angleTo position on turret to rotate to
     * @param inTicks true if angle is in ticks, false if in degrees
     * 
     */
    public RotateTurret(Turret turret, double targetAngle) {
        this.turret = turret;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(turret, HealthState.GREEN);
        if (targetAngle > turret.SOFT_MAXIMUM_DEGREES || targetAngle < turret.SOFT_MINIMUM_DEGREES) {
            throw new IllegalArgumentException("Invalid angle: must be within range " + turret.SOFT_MINIMUM_DEGREES
                    + " and " + turret.SOFT_MAXIMUM_DEGREES);
        }
        angle = targetAngle;
        ERROR_MARGIN = Turret.turretConfig.getAllowedError();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        turret.setTargetAngleInDegrees(angle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angle - turret.getCurrentAngleInDegrees()) < ERROR_MARGIN;
    }

    @Override
    public void end(boolean interrupted) {
        turret.moveByPercentOutput(0);
    }

}