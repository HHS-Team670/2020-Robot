package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.commands.vision.GetVisionData;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

public class UpdatePoseFromVision extends SequentialCommandGroup implements MustangCommand {

    private DriveBase driveBase;
    private MustangCoprocessor coprocessor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public UpdatePoseFromVision(DriveBase driveBase, MustangCoprocessor pi){
        this.driveBase = driveBase;
        this.coprocessor = pi;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        addCommands(
            new GetVisionData(coprocessor)
        );
    }

    @Override
    public void initialize() {
        double distanceFromTargetMeters = coprocessor.getDistanceToTargetCm() * 100;
        double angleFromTargetForwardDegrees = coprocessor.getAngleToTargetPerpendicular();
        double xToTargetForward = distanceFromTargetMeters * Math.sin(Math.toRadians(angleFromTargetForwardDegrees));
        double yToTargetForward = distanceFromTargetMeters * Math.cos(Math.toRadians(angleFromTargetForwardDegrees));
        // Todo need to check direction?
        double currentX = FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS + xToTargetForward;
        Translation2d updatedLoc = new Translation2d(currentX, yToTargetForward);

        // driveBase.resetOdometry(pose);

    }

    // This should basically be an instant command
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}