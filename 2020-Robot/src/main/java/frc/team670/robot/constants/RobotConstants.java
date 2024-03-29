package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.team670.mustanglib.constants.RobotConstantsBase;

public class RobotConstants extends RobotConstantsBase{

    // Robot Dimensions in Inches
    public static final double ROBOT_LENGTH = 29.5, ROBOT_WIDTH = 30.3, DRIVEBASE_TO_GROUND = 2.03;

    public static final double ROBOT_FULL_LENGTH_WITH_BUMPER = 36;

    public static final double TURRET_CENTER_TO_FRONT = 20;

    // Needed for vision. In meters
    public static final double TURRET_CAMERA_HEIGHT = 20; //TODO: FIGURE THIS OUT

    // Drive Base Gearing
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which
                                                            // one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;

    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1 / DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER
            * Math.PI;

    /** The number of ticks per inch of wheel travel */
    public static final int DIO_TICKS_PER_INCH = (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));



    /** The number of meters per roatation of a drivebase wheel */
    public static final double DRIVEBASE_METERS_PER_ROTATION = (1 / DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER
            * Math.PI * 0.0254;

    public static final double INDEXER_WIDTH = 203.2; // in mm
    public static final double SENSOR_TO_BALL = 127; // the distance from ToF sensor to middle of the ball, in mm
    public static final double MIN_BALL_DETECTED_WIDTH_INDEXER = 70;
    public static final double MIN_BALL_DETECTED_WIDTH_TUNNEL = 70;

    // Talon PID Constants
    public static final int kTimeoutMs = 0;

    public static final double leftKsVolts = 0.246;
    public static final double leftKvVoltSecondsPerMeter = 2.1;
    public static final double leftKaVoltSecondsSquaredPerMeter = 0.2;
    public static final double rightKsVolts = 0.12;
    public static final double rightKvVoltSecondsPerMeter = 2.1;
    public static final double rightKaVoltSecondsSquaredPerMeter = 0.1;

    // "WHEEL_BASE" is really track width
    public static final double kTrackwidthMeters = 0.702;

    // VISION Constants

    public static final int VISION_ERROR_CODE = -99999;
    public static final double kHorizontalFOV = 61; // degrees
    public static final double kVerticalFOV = 34.3; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double TILT_ANGLE = 13; //degrees
    public static final double CAMERA_HEIGHT = 0.9525; //meters

    // Autonomous Constants

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    public static final double leftKPDriveVel = 6;
    public static final double leftKIDriveVel = 0.00;
    public static final double leftKDDriveVel = 0.0;

    public static final double rightKPDriveVel = 2.4;
    public static final double rightKIDriveVel = 0.00;
    public static final double rightKDDriveVel = 0.0;

    public static final double kMaxSpeedInchesPerSecond = 12;
    public static final double kMaxAccelerationInchesPerSecondSquared = 12;

    public static final double kMaxSpeedMetersPerSecond = 1.2;// 1; //0.305;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.22;// 1; //0.305;

    public static final double kMaxSpeedMetersPerSecond2 = 0.6;// 1; //0.305;
    public static final double kMaxAccelerationMetersPerSecondSquared2 = 0.6;// 1; //0.305;
    public static final double endVelocityMetersPerSecond2 = 0.4;

    public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints = new DifferentialDriveKinematicsConstraint(
            kDriveKinematics, kMaxSpeedMetersPerSecond2);

        public static final DifferentialDriveKinematicsConstraint kAutoPathConstraintsIntaking = new DifferentialDriveKinematicsConstraint(
                kDriveKinematics, kMaxSpeedMetersPerSecond);

    //The turret angles depending on the robot start position 
    public static final double leftTurretAng = 0;
    public static final double rightTurretAng = -25;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;
    public static final boolean kNavXReversed = true;

}