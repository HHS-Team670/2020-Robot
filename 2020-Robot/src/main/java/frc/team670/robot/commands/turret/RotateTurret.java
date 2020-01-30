package frc.team670.robot.commands.turret;
import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Turret;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateTurret extends MustangCommandBase {
    private Turret turret;

    public RotateTurret(Turret turret) {

        this.turret = turret;
        

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        
        return null;
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
        
    }
        
}