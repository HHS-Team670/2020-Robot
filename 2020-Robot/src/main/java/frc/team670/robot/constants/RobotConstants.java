package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;

public class RobotConstants {

    // Robot Dimensions in Inches
    public static final double ROBOT_LENGTH = 29.5, ROBOT_WIDTH = 30.3, DRIVEBASE_TO_GROUND = 2.03;

    public static final double ROBOT_FULL_LENGTH_WITH_BUMPER = 36;

    public static final double TURRET_CENTER_TO_FRONT = 20;

    // Drive Base Gearing
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which
                                                            // one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;

    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1 / DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER
            * Math.PI;

    /**
     * The number of ticks per rotation of a drivebase wheel for the DIO Encoders
     */
    public static final int DIO_TICKS_PER_ROTATION = 1024;

    /** The number of ticks per inch of wheel travel */
    public static final int DIO_TICKS_PER_INCH = (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));

    /**
     * The number of ticks per rotation of a drivebase wheel for the SPARK Encoders
     */
    public static final int SPARK_TICKS_PER_ROTATION = 1024;

    /** The number of meters per roatation of a drivebase wheel */
    public static final double DRIVEBASE_METERS_PER_ROTATION = (1 / DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER
            * Math.PI * 0.0254;

    // Talon PID Constants
    public static final int kTimeoutMs = 0;

    public static final double rightKsVolts = 0.208;
    public static final double rightKvVoltSecondsPerMeter = 2.2;
    public static final double rightKaVoltSecondsSquaredPerMeter = 0.578;

    public static final double leftKsVolts = 0.208;
    public static final double leftKvVoltSecondsPerMeter = 2.2;
    public static final double leftKaVoltSecondsSquaredPerMeter = 0.578;

    // "WHEEL_BASE" is really track width
    public static final double kTrackwidthMeters = 0.702;

    // VISION Constants

    public static final int VISION_ERROR_CODE = -99999;

    // Autonomous Constants

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    public static final double leftKPDriveVel = 2.7;
    public static final double leftKIDriveVel = 0.00;
    public static final double leftKDDriveVel = 0.0;




    public static final double rightKPDriveVel = 0.09;
    public static final double rightKIDriveVel = 0.00;
    public static final double rightKDDriveVel = 0.0;
    public static final double kMaxSpeedInchesPerSecond = 12;
    public static final double kMaxAccelerationInchesPerSecondSquared = 12;

    public static final double kMaxSpeedMetersPerSecond = 1.22;// 1; //0.305;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.22;// 1; //0.305;

    public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints = new DifferentialDriveKinematicsConstraint(
            kDriveKinematics, kMaxSpeedMetersPerSecond);

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;
    public static final boolean kNavXReversed = true;

}