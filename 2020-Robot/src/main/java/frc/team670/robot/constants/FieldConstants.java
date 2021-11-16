package frc.team670.robot.constants;

/**
 * All values in meters unless otherwise specified
 */
public class FieldConstants {

    public static final double FIELD_ORIGIN_TO_OUTER_GOAL_CENTER_X_METERS = 2.404364;
    public static final double FLOOR_BAR_TO_CLIMBER_BAR_METERS = 0.809625;
    public static final double TRENCH_BALL_CENTER_FROM_SIDE_WALL_METERS = 0.70485;
    public static final double TARGET_WALL_TO_BASELINE = 3.556; // Turret center to target wall distance
    public static final double EDGE_OF_BASELINE = TARGET_WALL_TO_BASELINE + 0.0508;

    public static final double TARGET_CENTER_HEIGHT = 2.495;
    public static final double VISION_TARGET_CENTER_HEIGHT = 2.24;

    public static final double FAR_TARGET_X_POS = Units.feetToMeters(54);
    public static final double FAR_TARGET_Y_POS =
          Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final Pose2d FAR_TARGET_POSE =
          new Pose2d(new Translation2d(kFarTgtXPos, kFarTgtYPos), new Rotation2d(0.0));

    public static final Pose2d TARGET_POSE = new Pose2d(0, -2.4, Rotation2d.fromDegrees(0)); 


    

}