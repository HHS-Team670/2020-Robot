package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Intake;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class RunIntakeConveyor extends ParallelCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;
    private Conveyor conveyor;

    public RunIntakeConveyor(Intake intake, Conveyor conveyor, boolean isReversed){
        this.intake = intake;
        this.conveyor = conveyor;
        addCommands(
            new RunIntake(isReversed, intake),
            new RunConveyor(isReversed, conveyor)
        );
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        conveyor.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}