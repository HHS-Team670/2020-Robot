package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class StartShooterByDistance extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private MustangCoprocessor coprocessor;
    private double targetRPM;
    private double distanceToTarget;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public StartShooterByDistance(Shooter shooter, MustangCoprocessor pi){
        this.shooter = shooter;
        this.coprocessor = pi;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        distanceToTarget = coprocessor.getDistanceToTargetCm();
        targetRPM = shooter.getTargetRPMForDistance(distanceToTarget);
        shooter.setVelocityTarget(targetRPM);
        shooter.setRampRate(true);
    }

    @Override
    public void execute(){
        shooter.run();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}