package frc.team670.robot.commands;

import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.MustangSubsystemBase;

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
        // PIDController leftPIDController = new PIDController(SmartDashboard.getNumber("leftkP", 0), RobotConstants.leftKIDriveVel, SmartDashboard.getNumber("leftKD", 0));
        // PIDController rightPIDController = new PIDController(SmartDashboard.getNumber("rightkP", 0), RobotConstants.rightKIDriveVel, SmartDashboard.getNumber("rightKD", 0));
        
        // // TODO change this: should return cmd group that zeros and then follows path
        // path.reset();
        // return new RamseteCommand(path.getTrajectory(), driveBase::getPose,
        //     new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
        //     new SimpleMotorFeedforward(RobotConstants.ksVolts, RobotConstants.kvVoltSecondsPerMeter,
        //         RobotConstants.kaVoltSecondsSquaredPerMeter),
        //     RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController, rightPIDController,
        //     // RamseteCommand passes volts to the callback
        //     driveBase::tankDriveVoltage, driveBase);

     //public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
    // TODO: This uses the left feedforward constants, but applies to all motors... Should change this to reflect all motors
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(RobotConstants.leftKsVolts,
                                       RobotConstants.leftKvVoltSecondsPerMeter,
                                       RobotConstants.leftKaVoltSecondsSquaredPerMeter),
            RobotConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(RobotConstants.kMaxSpeedMetersPerSecond,
                             RobotConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(RobotConstants.kDriveKinematics)
            // Apply the voltage constraint
            // .addConstraint(RobotConstants.kAutoPathConstraints)
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 0),
            new Translation2d(2, -0),
            new Translation2d(3, -0),
            new Translation2d(4, -0),
            new Translation2d(5, -0)

        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(6, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    // Paste this variable in
    RamseteController disabledRamsete = new RamseteController() {
    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
            double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
    }
    };

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveBase::getPose,
        disabledRamsete,
        new SimpleMotorFeedforward(RobotConstants.leftKsVolts,
                                   RobotConstants.leftKvVoltSecondsPerMeter,
                                   RobotConstants.leftKaVoltSecondsSquaredPerMeter
                                   ),
        new SimpleMotorFeedforward(RobotConstants.rightKsVolts,
                                   RobotConstants.rightKvVoltSecondsPerMeter,
                                   RobotConstants.rightKaVoltSecondsSquaredPerMeter
                                   ),
        RobotConstants.kDriveKinematics,
        driveBase::getWheelSpeeds,
        new PIDController(SmartDashboard.getNumber("leftkP", 0), RobotConstants.leftKIDriveVel, SmartDashboard.getNumber("leftKD", 0)),
        new PIDController(SmartDashboard.getNumber("rightkP", 0), RobotConstants.rightKIDriveVel, SmartDashboard.getNumber("rightKD", 0)),
        // RamseteCommand passes volts to the callback
        driveBase::tankDriveVoltage,
        driveBase
    );

    // Reset odometry to the starting pose of the trajectory.
    driveBase.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand;
  }
    }
