package frc.team670.robot.commands.turret;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Rotates the turret to an angle with the goal of having the turret pointed in
 * the direction of the target for as much as possible.
 * 
 * @author ctychen
 */
public class RotateTurret extends CommandBase implements MustangCommand {

    private Turret turret;
    private DriveBase driveBase;
    private MustangCoprocessor coprocessor;
    private double targetAngle;
    private double heading;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public RotateTurret(Turret turret, DriveBase driveBase, MustangCoprocessor pi) {
        this.turret = turret;
        this.driveBase = driveBase;
        this.coprocessor = pi;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(turret, HealthState.GREEN);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    /**
     * Gets the relative angle to the target from camera when possible, converts to
     * absolute angle, and sets that to the turret's target position.
     * 
     * If vision data is unavailable, use position on the field to determine the
     * approximate angle to the target and account for heading to turn.
     * 
     */
    @Override
    public void initialize() {
        double relativeAngleToTarget = coprocessor.getAngleToTarget();
        // When vision can't detect the target, rotates the turret based on odometry
        if (relativeAngleToTarget == RobotConstants.VISION_ERROR_CODE) {
            double currentX = driveBase.getPose().getTranslation().getX();
            double currentY = driveBase.getPose().getTranslation().getY();
            // Angle from known position on field to center of outer goal/vision target
            double drivebasePosToGoalAngle = Math.toDegrees(
                    Math.atan((currentX - FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS) / currentY));
            // TODO: Need to figure out this, need to account for heading
            relativeAngleToTarget = drivebasePosToGoalAngle;
        }
        targetAngle = turret.relativeAngleToAbsoluteInDegrees(relativeAngleToTarget);
        turret.setSystemTargetAngleInDegrees(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return turret.hasReachedTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        turret.moveByPercentOutput(0);
    }

    public double getDrivebaseAngle() {
        return driveBase.getHeading();
    }

}