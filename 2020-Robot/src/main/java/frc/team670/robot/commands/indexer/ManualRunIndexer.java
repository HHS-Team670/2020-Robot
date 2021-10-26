package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;

/**
 * Empties the revolver by running motors backward 3 chambers
 * 
 * @author palldas
 */
public class ManualRunIndexer extends CommandBase implements MustangCommand {

    private Indexer indexer;
    private Intake intake;
    private Conveyor conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean reversed;

    public ManualRunIndexer(Indexer indexer, Conveyor conveyor, Intake intake, boolean reversed) {
        this.indexer = indexer;
        this.conveyor = conveyor;
        this.intake = intake;
        addRequirements(indexer);
        this.reversed = reversed; // up is false, down is true
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.YELLOW); // can work if all but the exit sensor is down
        // can keep moving motors until the sensor doesn't sense anymore balls, or
        // rotate until u know that 3 ball've been shot
    }

    /**
     * Move the indexer to a "staging" position so the updraw can spin up to speed
     */
    @Override
    public void initialize() {
        Logger.consoleLog("Preparing to empty indexer");
        indexer.updraw(false);
    }

    /**
     * Spin the updraw wheels
     */
    @Override
    public void execute() {
        if(!reversed && indexer.updrawIsUpToSpeed()){
            indexer.run(true);
        }
        else {
            indexer.runReversed(true);
        }
        conveyor.run(reversed);
        intake.roll(reversed);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.consoleLog("Indexer system emptied");
        indexer.stopUpdraw();
        indexer.stop();
        intake.stop();
        conveyor.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }

}