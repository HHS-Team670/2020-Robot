package frc.team670.robot.commands.climb;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.paths.climb.FloorBarToAlignClimb;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

/**
 * Once the driver aligns the back wheels to the bars under the generator, 
 * drives straight to align with the climbing bar and extends the climber when in position.
 */
public class DriveToBarAndPrepareClimb extends SequentialCommandGroup implements MustangCommand {
    
    private Climber climber;
    private DriveBase driveBase;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * Initializes this command from the given parameters
     * 
     * @param driveBase the drivebase of the robot
     * @param climber the climber of the robot
     */
    public DriveToBarAndPrepareClimb(DriveBase driveBase, Climber climber) {
        this.driveBase = driveBase;
        this.climber = climber;
        addRequirements(driveBase, climber);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
        healthReqs.put(driveBase, HealthState.GREEN);
        if (driveBase.isAlignedOnFloorBars()){
            addCommands(
                getTrajectoryFollowerCommand(new FloorBarToAlignClimb(driveBase), driveBase)
            );
        }
    }

    /**
     * Schedules an extend climber command after this command completes
     * 
     * @param interrupted whether the command was interrupted or completed
     */
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