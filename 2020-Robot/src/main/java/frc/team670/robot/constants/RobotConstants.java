package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 29.5, ROBOT_WIDTH = 30.3, DRIVEBASE_TO_GROUND = 2.03;

    // Drive Base Gearing
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which
                                                            // one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;
    
    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1/DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER * Math.PI;
    
    /** The number of ticks per rotation of a drivebase wheel for the DIO Encoders  */
    public static final int DIO_TICKS_PER_ROTATION = 1024;
    
    /** The number of ticks per inch of wheel travel */
    public static final int DIO_TICKS_PER_INCH = (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));
    
    /** The number of ticks per rotation of a drivebase wheel for the SPARK Encoders  */
    public static final int SPARK_TICKS_PER_ROTATION = 1024;

    //VISION Constants
    public static final double WHEEL_BASE = 25.662; //measured 1/26/19

    /** The number of meters per roatation of a drivebase wheel */
    public static final double DRIVEBASE_METERS_PER_ROTATION = (1/DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER * Math.PI * 0.0254;

    //Talon PID Constants
    public static final int kTimeoutMs = 0;

    public static final double ksVolts = 0.11; //0.224;
    public static final double kvVoltSecondsPerMeter = 2.27; //2.22;
    public static final double kaVoltSecondsSquaredPerMeter = 0.765; // 0.715;

    // "WHEEL_BASE" is really track width
    public static final double kTrackwidthInches = WHEEL_BASE;
    public static final double kTrackwidthMeters = WHEEL_BASE * 0.0254;

    public static final DifferentialDriveKinematics kDriveKinematics =
    new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kPDriveVel = 4;
    public static final double kIDriveVel = 0.00;
    public static final double kDDriveVel = 0.0;
    public static final double kMaxSpeedInchesPerSecond = 12;
    public static final double kMaxAccelerationInchesPerSecondSquared = 12;

    public static final double kMaxSpeedMetersPerSecond = 0.75;//1; //0.305;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;//1; //0.305;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;
    public static final boolean kNavXReversed = true;

}