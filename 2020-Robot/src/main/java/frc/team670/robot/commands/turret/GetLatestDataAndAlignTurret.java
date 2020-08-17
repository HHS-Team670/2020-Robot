package frc.team670.robot.commands.turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.vision.GetVisionData;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Gets the latest vision data and aligns the turret toward the vision target
 * 
 * @author lakshbhambhani
 */
public class GetLatestDataAndAlignTurret extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * Initializes this command from the given parameters
     * @param turret the turret of the robot
     * @param driveBase the drivebase of the robot
     * @param pi the raspberry pi
     */
    public GetLatestDataAndAlignTurret(Turret turret, DriveBase driveBase, MustangCoprocessor pi){
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