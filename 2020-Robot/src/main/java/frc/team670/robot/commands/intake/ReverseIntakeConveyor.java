package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Intake;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class ReverseIntakeConveyor extends ParallelCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;
    private Conveyor conveyor;

    public ReverseIntakeConveyor(Intake intake, Conveyor conveyor){
        this.intake = intake;
        this.conveyor = conveyor;
        addCommands(
            new RunIntake(true, intake),
            new RunConveyor(true, conveyor)
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