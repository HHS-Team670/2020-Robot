package frc.team670.robot.commands.turret;

import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.commands.joystick_controls.JoystickTurret;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.subsystems.Turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateTurret extends MustangCommandBase {
    private Turret turret;
    private double initialPos;
    private double angle;
    private double generalSpeed;
    private double ERROR_MARGIN = 2;

    public RotateTurret(double angleTo, Turret turret) {
        this.turret = turret;
        initialPos = turret.getEncoderPos();
        //turret.setZeroPoint(this.turret.getEncoderPos());
        //initialPos = this.turret.getZeroPoint();
        this.angle = angleTo;
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