package frc.team670.robot.commands.turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

/**
 * Automatically rotates the turret with the goal of having the turret pointed
 * in the direction of the target for as much as possible, whether this is done
 * with vision or from robot heading/pose, the point is to keep it roughly
 * tracking
 * 
 * Bonus points for being written on a plane
 * 
 * @author ctychen
 */
public class AutoRotate extends CommandBase implements MustangCommand {

    private Turret turret;
    private DriveBase driveBase;
    private MustangCoprocessor coprocessor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public AutoRotate(Turret turret, MustangCoprocessor pi, DriveBase driveBase) {
        this.turret = turret;
        this.driveBase = driveBase;
        this.coprocessor = pi;
        addRequirements(turret);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(turret, HealthState.GREEN);
        healthReqs.put(driveBase, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        // Set current pose?
        Logger.consoleLog("Turret auto rotate init");
    }

    @Override
    public void execute() {
        double relativeAngleToTarget = turret.getCurrentAngleInDegrees();
        // Attempt to use vision if it's enabled
        double currentX = driveBase.getPose().getTranslation().getX();
        double currentY = driveBase.getPose().getTranslation().getY();
        // // Angle from known position on field to center of outer goal/vision target
        double drivebasePosToGoalAngle = Math.toDegrees(
                Math.atan((currentX - FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS) / currentY));
        Logger.consoleLog("Drivebase pos to goal angle: %s", drivebasePosToGoalAngle);
        double heading = driveBase.getHeading();
        Logger.consoleLog("Drivebase heading: %s", heading);
        // zero degrees = pointing straight forwards, +180 clockwise, -180 counterclockwise
        // this may not be how to use heading
        relativeAngleToTarget = drivebasePosToGoalAngle - heading;
        Logger.consoleLog("Turret target relative angle is %s", relativeAngleToTarget);
        double targetAngle = turret.relativeAngleToAbsoluteInDegrees(relativeAngleToTarget);
        Logger.consoleLog("Turret target absolute angle is %s", targetAngle);
        turret.setSystemTargetAngleInDegrees(targetAngle);
    }

    /**
     * This command should run constantly, so this will always be false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}