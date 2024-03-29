package frc.team670.robot.commands.turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Turret;

/**
 * Rotate the turret to the reverse limit in order to zero the encoder. 
 */
public class RotateToAngle extends CommandBase implements MustangCommand {

    private Turret turret;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private double angle;

    public RotateToAngle(Turret turret, double angle) {
        this.turret = turret;
        this.angle = angle;
        addRequirements(turret);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(turret, HealthState.GREEN);
    }

    @Override
    public void execute(){
        turret.setSystemTargetAngleInDegrees(angle);
    }

    @Override
    public boolean isFinished() {
        return turret.hasReachedTargetPosition();
    }

    @Override
    public void end(boolean interrupted){
        turret.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}