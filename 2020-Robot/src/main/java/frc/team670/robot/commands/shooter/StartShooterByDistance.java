package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

/**
 * Starts the shooter based on the distance to the target
 */
public class StartShooterByDistance extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private DriveBase driveBase;
    private double targetRPM;
    private double distanceToTarget;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    /**
     * Initializes this command from the given parameters
     * @param shooter the shooter of the robot
     * @param driveBase the drivebase of the robot
     */
    public StartShooterByDistance(Shooter shooter, DriveBase driveBase){
        this.shooter = shooter;
        this.driveBase = driveBase;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        double currentX = driveBase.getPose().getTranslation().getX();
        double currentY = driveBase.getPose().getTranslation().getY();
        double distanceToTarget = Math.sqrt(
            (Math.pow(currentX - FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS, 2) +
             Math.pow(currentY, 2))
        );
        Logger.consoleLog("Shooter distance to target %s", distanceToTarget);
        targetRPM = shooter.getTargetRPMForDistance(SmartDashboard.getNumber("distance", 3));
        Logger.consoleLog("Shooter Stage 2 RPM should be %s", targetRPM);
        // shooter.setVelocityTarget(targetRPM);
        // shooter.setRampRate(true);
    }

    @Override
    public void execute(){
        shooter.run();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}