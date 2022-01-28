package frc.team670.robot.commands.climb;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Once the driver aligns the back wheels to the bars under the generator, 
 * drives straight to align with the climbing bar and extends the climber when in position.
 */
public class DriveToBarAndPrepareClimb extends SequentialCommandGroup implements MustangCommand {
    
    private Climber climber;
    private DriveBase driveBase;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public DriveToBarAndPrepareClimb(DriveBase driveBase, Climber climber) {
        this.driveBase = driveBase;
        this.climber = climber;
        addRequirements(driveBase, climber);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
        healthReqs.put(driveBase, HealthState.GREEN);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            addCommands(
                new ExtendClimber(climber)
            );
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}