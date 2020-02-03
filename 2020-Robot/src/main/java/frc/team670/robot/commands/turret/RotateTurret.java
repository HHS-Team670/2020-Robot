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
    public RotateTurret(double angleTo, Turret turret, boolean inTicks) {
        this.turret = turret;
        //initialPos = turret.getEncoderPos();
        if (!inTicks) {
            if (angleTo > 360) {
                throw new IllegalArgumentException("Cannot use angle greater than 360 degrees");
            }
            this.angle = angleTo;
        } else {
            if (angleTo > turret.TICKS_PER_REVOLUTION) {
                throw new IllegalArgumentException("Cannot use tick measure greater than " + turret.TICKS_PER_REVOLUTION);
            }
            this.angle = turret.getDegrees(angleTo);
        }
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> health = new HashMap<MustangSubsystemBase, HealthState>();
        health.put(turret, turret.getHealth(true));
        return health;
    }
 
    @Override
    public void initialize() {
        if (Math.abs(angle - turret.getAngleInDegrees()) > ERROR_MARGIN) {
            turret.setTurretSpeed(generalSpeed);
        }
    }

    @Override
    public void execute() {
        if (angle - turret.getAngleInDegrees() < -ERROR_MARGIN) {
            turret.setTurretSpeed(generalSpeed);

        } else if (angle - turret.getAngleInDegrees() > ERROR_MARGIN) {
            turret.setTurretSpeed(-generalSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        
        return Math.abs(angle - turret.getAngleInDegrees()) < ERROR_MARGIN;
    }

    @Override
    public void end(boolean interrupted) {
        
        turret.setTurretSpeed(0);
    }
        
}