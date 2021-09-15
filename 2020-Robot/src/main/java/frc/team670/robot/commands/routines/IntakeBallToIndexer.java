package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.indexer.RunIndexer;
import frc.team670.robot.commands.intake.DeployIntake;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;

public class IntakeBallToIndexer extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public IntakeBallToIndexer(Intake intake, Conveyor conveyor, Indexer indexer) {
        addRequirements(intake, conveyor, indexer);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(indexer, HealthState.GREEN);
        if (!intake.isDeployed()) {
            addCommands(new DeployIntake(true, intake));
        }
        addCommands(
            new ParallelCommandGroup(
                new RunIntake(false, intake), 
                new RunIndexer(indexer)
            )
        );
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }


}