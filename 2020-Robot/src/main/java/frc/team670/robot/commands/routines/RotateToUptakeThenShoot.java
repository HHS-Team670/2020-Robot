package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class RotateToUptakeThenShoot extends CommandGroupBase implements MustangCommand {

    private Indexer indexer;
    private Shooter shooter;
    //private Turret turret;

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public RotateToUptakeThenShoot(Indexer indexer, Shooter shooter){
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer, shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void addCommands(Command... commands) {
        // Rotate indexer to prepare to uptake
        // Spin up shooter 
        // (turn turret)
    }
    
}