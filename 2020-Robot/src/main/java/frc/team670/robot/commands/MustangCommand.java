package frc.team670.robot.commands;

import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;

import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

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
     * @param trajectory The trajectory to follow
     * @param driveBase 
     * @return A RamseteCommand which will drive the given trajectory
     */
    default RamseteCommand getTrajectoryFollowerCommand(Trajectory trajectory, DriveBase driveBase){
        PIDController leftPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
        RobotConstants.kDDriveVel);
        PIDController rightPIDController = new PIDController(RobotConstants.kPDriveVel, RobotConstants.kIDriveVel,
        RobotConstants.kDDriveVel);
        return new RamseteCommand(trajectory, driveBase::getPose,
            new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
            new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
                RobotConstants.kaVoltSecondsSquaredPerMeter),
            RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController, rightPIDController,
            // RamseteCommand passes volts to the callback
            driveBase::tankDriveVoltage, driveBase);
    }

}