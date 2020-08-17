package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Rotates indexer in reverse and then acts as if intaking (to prevent further jams) to unjam the indexer
 * 
 * @author ctychen
 */
public class UnjamIndexer extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private double startTime;

    private int rotateForwardTimes;

    /**
     * Initializes this command from the given indexer
     * @param indexer the indexer of the robot
     */
    public UnjamIndexer(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
        indexer.setRotatorMode(true);
        rotateForwardTimes = 0;
    }

    @Override
    public void execute() {
        if(rotateForwardTimes <= 150){
            indexer.moveByPercentOutput(-0.4);
            rotateForwardTimes++;
        }
        else{
            indexer.moveByPercentOutput(0.4);
        }
    }

    /**
     * @return true when the top chamber of the indexer is empty (ball has been
     *         updrawed)
     */
    @Override
    public boolean isFinished() {
        return Math.abs(startTime - System.currentTimeMillis()) > 500;
    }

    @Override
    public void end(boolean interrupted){
        indexer.setRotatorMode(false);
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}