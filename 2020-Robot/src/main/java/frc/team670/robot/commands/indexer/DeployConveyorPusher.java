package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class DeployConveyorPusher extends InstantCommand implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean deploy;
    private Indexer indexer;

    public DeployConveyorPusher(boolean isDeploy, Indexer indexer) {
        this.deploy = isDeploy;
        this.indexer = indexer;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        addRequirements(indexer);
    }

    public void initialize() {
        indexer.deployPusher(deploy);
    }

    // Called once after isFinished returns true
    public void end() {
    }

    public void interrupted() {
        end();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
