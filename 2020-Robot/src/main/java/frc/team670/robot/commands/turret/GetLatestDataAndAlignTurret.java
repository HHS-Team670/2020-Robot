package frc.team670.robot.commands.turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.commands.vision.GetVisionData;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

/**
 * @author lakshbhambhani
 */

public class GetLatestDataAndAlignTurret extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public GetLatestDataAndAlignTurret(Turret turret, DriveBase driveBase, Vision pi){
        addRequirements(turret);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(turret, HealthState.GREEN);

        addCommands(
                new GetVisionData(pi, driveBase),
                new RotateTurret(turret, driveBase, pi)
        );
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return this.healthReqs;
    }
    
}