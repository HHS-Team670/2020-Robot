package frc.team670.robot.commands.turret;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Rotates the turret to an angle determined by vision data
 * @author ctychen
 */
public class RotateTurretWithVision extends CommandBase implements MustangCommand {

    private Turret turret;
    private MustangCoprocessor coprocessor;
    private double targetAngle;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * @param angleTo position on turret to rotate to
     * @param inTicks true if angle is in ticks, false if in degrees
     * 
     */
    public RotateTurretWithVision(Turret turret, MustangCoprocessor pi) {
        this.turret = turret;
        this.coprocessor = pi;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(turret, HealthState.GREEN);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    /**
     * Gets the relative angle to the target from camera, converts to absolute
     * angle, and sets that to the turret's target position.
     */
    @Override
    public void initialize() {
        double angleToTarget = coprocessor.getAngleToTarget();
        targetAngle = turret.relativeAngleToAbsoluteInDegrees(angleToTarget);
        turret.setSystemTargetAngleInDegrees(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return turret.hasReachedTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        turret.moveByPercentOutput(0);
    }

}