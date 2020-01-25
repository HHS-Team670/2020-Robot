package frc.team670.robot.commands.turret;
import frc.team670.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateTurret extends CommandBase{
    private Turret turret;
    public RotateTurret(Turret turret){

       this.turret = turret;

    }
    
        
}