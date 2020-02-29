package frc.team670.robot.commands;

import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;

import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * Represents a robot action with defined health requirements for every subsystem it uses.
 * 
 * @author ctychen, lakshbhambhani
 */
public interface MustangCommand{

    /**
     * @return A Map containing the minimum health condition for each subsystem that this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

    /**
     * 
     * @param path The trajectory to follow
     * @param driveBase 
     * @return A RamseteCommand which will drive the given trajectory
     */
    default RamseteCommand getTrajectoryFollowerCommand(Path path, DriveBase driveBase){
        PIDController leftPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
        RobotConstants.kDDriveVel);
        PIDController rightPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
        RobotConstants.kDDriveVel);
        // TODO change this: should return cmd group that zeros and then follows path
        path.reset();
        return new RamseteCommand(path.getTrajectory(), driveBase::getPose,
            new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
            new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
                RobotConstants.kaVoltSecondsSquaredPerMeter),
            RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController, rightPIDController,
            // RamseteCommand passes volts to the callback
            driveBase::tankDriveVoltage, driveBase);
    }

}