package frc.team670.robot.commands.turret;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.joystick_controls.JoystickTurret;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Rotates the turret to a specified point
 */
public class RotateTurret extends CommandBase implements MustangCommand {
    private Turret turret;
    //private double initialPos;
    private double angle;
    private double generalSpeed;
    private double ERROR_MARGIN = 2;

    /**
     * @param angleTo position on turret to rotate to
     * @param inTicks true if angle is in ticks, false if in degrees
     * 
     */
    public RotateTurret(Turret turret, double targetAngle) {
        this.turret = turret;
            if (targetAngle > turret.SOFT_MAXIMUM_DEGREES || targetAngle < turret.SOFT_MINIMUM_DEGREES) {
                throw new IllegalArgumentException("Invalid angle: must be within range " + turret.SOFT_MINIMUM_DEGREES + " and " + turret.SOFT_MAXIMUM_DEGREES);
            }
            this.angle = angle;
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> health = new HashMap<MustangSubsystemBase, HealthState>();
        health.put(turret, turret.getHealth(true));
        return health;
    }
 
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        turret.setTargetAngleInDegrees(angle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angle - turret.getCurrentAngleInDegrees()) < ERROR_MARGIN;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTurretSpeed(0);
    }
        
}