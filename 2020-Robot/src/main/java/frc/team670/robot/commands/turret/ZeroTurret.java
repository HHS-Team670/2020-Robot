package frc.team670.robot.commands.turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Rotates the turret to an angle of 0
 * 
 * @author lakshbhambhani
 */
public class ZeroTurret extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Turret turret;

    /**
     * Initializes this command from the given turret
     * @param turret the turret of the robot
     */
    public ZeroTurret(Turret turret){
        this.turret = turret;
        addRequirements(turret);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(turret, HealthState.GREEN);
        addCommands(
                new RotateToHome(turret),
                new RotateToAngle(turret, 0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            turret.setZeroedAlready(true);
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }
    
}