package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.functions.MathUtils;

public class ToggleShooter extends InstantCommand implements MustangCommand {

    private Shooter shooter;
    private DriveBase driveBase;

    public ToggleShooter(Shooter shooter, DriveBase driveBase) {
        this.shooter = shooter;
        this.driveBase = driveBase;
    }

    @Override
    public void initialize() {
        super.initialize();
        if (MathUtils.doublesEqual(0.0, shooter.getStage2Velocity(), 10)) {
            double currentX = driveBase.getPose().getTranslation().getX();
            double currentY = driveBase.getPose().getTranslation().getY();
            double distanceToTarget = Math.sqrt(
                (Math.pow(currentX - FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS, 2) +
                 Math.pow(currentY, 2))
            );
            double targetRPM = shooter.getTargetRPMForDistance(distanceToTarget);
            shooter.setVelocityTarget(targetRPM);
            shooter.setRampRate(true);
            shooter.run();
        } else {
            shooter.stop();
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
    
}