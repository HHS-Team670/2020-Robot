package frc.team670.robot.commands.indexer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Stops the intake and conveyor, and takes the indexer out of intaking mode
 */
public class StopIntaking extends InstantCommand implements MustangCommand {

    private Indexer indexer;
    private Intake intake;
    private Conveyor conveyor;

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * Initializes this command with the given parameters
     * 
     * @param intake the intake of the robot
     * @param conveyor the conveyor of the robot
     * @param indexer the indexer of the robot
     */
    public StopIntaking(Intake intake, Conveyor conveyor, Indexer indexer) {
        this.indexer = indexer;
        this.intake = intake;
        this.conveyor = conveyor;
        addRequirements(intake, conveyor, indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(indexer, HealthState.GREEN);
        healthReqs.put(intake, HealthState.YELLOW);
        healthReqs.put(conveyor, HealthState.GREEN);

    }

    @Override
    public void initialize() {
        intake.stop();
        conveyor.stop();
        indexer.stopIntaking();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}