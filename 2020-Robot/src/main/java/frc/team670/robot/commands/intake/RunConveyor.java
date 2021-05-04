package frc.team670.robot.commands.intake;

import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

public class RunConveyor extends CommandBase implements MustangCommand {

    private Conveyor conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean reversed;

    public RunConveyor(boolean reversed, Conveyor conveyor) {
        this.conveyor = conveyor;
        this.reversed = reversed;
        addRequirements(conveyor);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(conveyor, HealthState.GREEN);
    }

    @Override
    public void execute() {
			conveyor.run(reversed);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}