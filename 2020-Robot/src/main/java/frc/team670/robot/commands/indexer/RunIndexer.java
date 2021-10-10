package frc.team670.robot.commands.indexer; 

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Indexer;

/**
 * Empties the revolver by running motors backward 3 chambers
 * 
 * @author palldas
 */
public class RunIndexer extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private long microSecondsSinceZeroBalls = -1;

    public RunIndexer(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.YELLOW); // can work if all but the exit sensor is down
        // can keep moving motors until the sensor doesn't sense anymore balls, or rotate until u know that 3 ball've been shot
    }

    /**
     * Move the indexer to a "staging" position so the updraw can spin up to speed
     */
    @Override
    public void initialize() {
        Logger.consoleLog("Preparing to empty indexer");
        indexer.updraw(false);
        microSecondsSinceZeroBalls = -1;
    }

    /**
     * Spin the updraw wheels
     */
    @Override
    public void execute() {
        if (indexer.updrawIsUpToSpeed()) {
            indexer.run(true);
        }
    }

    /**
     * @return true when the indexer has rotated to the position for staging the
     *         ball
     */
    @Override
    public boolean isFinished() {
        if(indexer.getTotalNumBalls() == 0 && microSecondsSinceZeroBalls == -1){
            microSecondsSinceZeroBalls =  RobotController.getFPGATime();
        }
        if(microSecondsSinceZeroBalls != -1 && RobotController.getFPGATime() - microSecondsSinceZeroBalls >= 500000){
            microSecondsSinceZeroBalls = -1;
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.consoleLog("Indexer system emptied");
        indexer.stopUpdraw();
        indexer.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }

}