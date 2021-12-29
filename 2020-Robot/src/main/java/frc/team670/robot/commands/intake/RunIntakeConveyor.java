package frc.team670.robot.commands.intake;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;

public class RunIntakeConveyor extends ParallelCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;
    private Conveyor conveyor;
    private Indexer indexer;

    public RunIntakeConveyor(Intake intake, Conveyor conveyor, Indexer indexer, boolean isReversed) {
        this.intake = intake;
        this.conveyor = conveyor;
        addCommands(
            new RunIntake(isReversed, intake),
            new RunConveyor(isReversed, conveyor, indexer)
        );
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}