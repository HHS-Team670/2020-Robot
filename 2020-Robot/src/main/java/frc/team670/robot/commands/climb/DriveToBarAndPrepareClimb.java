package frc.team670.robot.commands.climb;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

public class DriveToBarAndPrepareClimb extends SequentialCommandGroup implements MustangCommand {
    
    private Climber climber;
    private DriveBase driveBase;

    public DriveToBarAndPrepareClimb(DriveBase driveBase, Climber climber) {
        this.driveBase = driveBase;
        this.climber = climber;
        addRequirements(driveBase, climber);
        addCommands(
            // TODO: drive path from align against ground bars
            new ExtendClimber(climber)
        );

    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            addCommands(
                new Climb(climber)
            );
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}