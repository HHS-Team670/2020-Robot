package frc.team670.robot.commands.vision;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.functions.MathUtils;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class GetVisionData extends CommandBase implements MustangCommand {

    private Vision coprocessor;

    private long startTime;

    private boolean runVisionOnce;

    private DriveBase driveBase;

    private static final double MAX_TIME_TO_RUN = 2500; // Max time to run this in ms

    public GetVisionData(Vision coprocessor, DriveBase driveBase) {
        this.coprocessor = coprocessor;
        this.driveBase = driveBase;
    }

    @Override
    public void initialize() {
        runVisionOnce =  false;
        coprocessor.triggerVision();
        // SmartDashboard.putNumberArray(coprocessor.VISION_RETURN_NETWORK_KEY, new double[] { RobotConstants.VISION_ERROR_CODE,
        //         RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE }); // Clears vision data so we don't
        //         
        // coprocessor.clearLastValues();  
        // NetworkTableInstance.getDefault().flush();        // use old data accidentally
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if(!runVisionOnce && System.currentTimeMillis()-startTime > 800){
            runVisionOnce = true;
        }
    }

    @Override
    public boolean isFinished() {
        long time = System.currentTimeMillis();
        return (time > startTime + 100 || time > startTime + MAX_TIME_TO_RUN); //TODO add check for error code
    }

    @Override
    public void end(boolean interrupted) {
        double distanceFromTargetMeters = coprocessor.getDistanceToTargetMeters() * 100;
        double angleFromTargetForwardDegrees = coprocessor.getAngleToTarget();
        double xToTargetForward = distanceFromTargetMeters * Math.sin(Math.toRadians(angleFromTargetForwardDegrees));
        double yToTargetForward = distanceFromTargetMeters * Math.cos(Math.toRadians(angleFromTargetForwardDegrees));
        double currentX = FieldConstants.FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS + xToTargetForward;
        driveBase.resetOdometry(new Pose2d(currentX, yToTargetForward, Rotation2d.fromDegrees(driveBase.getHeading())));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}