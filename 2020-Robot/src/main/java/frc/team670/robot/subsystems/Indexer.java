package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.TimeOfFlightSensor;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.motorcontroller.MotorConfig;
import frc.team670.robot.utils.motorcontroller.TalonSRXLite;

/**
 * Represents the ball indexer subsystem, which tracks and stores up to 5 balls.
 * 
 * @author ctychen, eddieli, ruchidixit
 */
public class Indexer extends SparkMaxRotatingSubsystem {

    private TalonSRXLite updraw;

    private CANEncoder encoder;

    private TimeOfFlightSensor indexerIntakeSensor;
    private DutyCycleEncoder revolverAbsoluteEncoder;

    /*
     * Ranges (in mm) from the TOF sensor for which we know the ball was fully
     * intaked into the bottom chamber.
     */
    private int TOF_BALL_IN_MIN_RANGE = 15;
    private int TOF_BALL_IN_MAX_RANGE = 40;

    private boolean[] chamberStates;
    private double updrawCurrent;
    private double updrawPreviousCurrent;

    private double indexerCurrent;
    private double indexerPreviousCurrent;

    private boolean indexerIsJammed;

    private boolean ballIsUpdrawing;

    // For testing purposes
    private final double UPDRAW_SPEED = 0.3;

    // Current control: updraw
    // TODO: find all these values
    private static final double UPDRAW_CURR_P = 0.0;
    private static final double UPDRAW_CURR_I = 0;
    private static final double UPDRAW_CURR_D = 0;
    private static final double UPDRAW_CURR_FF = 0.2;

    // TODO: find these values
    private static final double UPDRAW_SHOOT_CURRENT_CHANGE_THRESHOLD = 0;
    private static final double UPDRAW_SHOOT_COMPLETED_CURRENT_CHANGE = 0;

    private static final int UPDRAW_CURRENT_SLOT = 0;

    private static final double UPDRAW_V_P = 0;
    private static final double UPDRAW_V_I = 0;
    private static final double UPDRAW_V_D = 0;
    private static final double UPDRAW_V_FF = 0.2;

    private static final int UPDRAW_VELOCITY_SLOT = 1;

    // TODO: Set these
    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 0;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 0;

    private static final double INDEXER_DEGREES_PER_CHAMBER = 72;

    // Bottom (intake position) is 0, for chamber 0.
    private static final int CHAMBER_0_AT_TOP_POS_IN_DEGREES = 180; // Shooting position for chamber 0
    private static final int CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES = 0; // Intaking position for chamber 0

    /**
     * PID and SmartMotion constants for the indexer rotator go here.
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.INDEXER_ROTATOR;
        }

        public int getSlot() {
            return 0;
        }

        public MotorConfig.Motor_Type getMotorType() {
            return MotorConfig.Motor_Type.NEO_550;
        }

        public double getP() {
            return 0.000;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
        }

        public double getFF() { // TODO: tune
            return 0;
        }

        public double getIz() {
            return 0;
        }

        public double getMaxOutput() {
            return 1;
        }

        public double getMinOutput() {
            return -1;
        }

        public double getMaxVelocity() {
            return 2100; // assuming 1 indexer rotation per second
        }

        public double getMinVelocity() {
            return 0;
        }

        public double getMaxAcceleration() {
            return 2100;
        }

        public double getAllowedError() {
            // return 0.0243;
            // 0.25 degree minimum from count / 360 deg per rotation
            return (0.25 / 360) * getRotatorGearRatio();
        }

        public boolean enableSoftLimits() {
            return false;
        }

        public float[] setSoftLimits() {
            return null;
        }

        public int getContinuousCurrent() {
            return 3;
        }

        public int getPeakCurrent() {
            return 6;
        }

        @Override
        public double getRotatorGearRatio() {
            return 35; 
        }

    }

    public static final Config INDEXER_CONFIG = new Config();

    public Indexer() {
        super(INDEXER_CONFIG);

        this.updraw = new TalonSRXLite(RobotMap.UPDRAW_SPINNER);
        this.indexerIntakeSensor = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_PORT);
        this.revolverAbsoluteEncoder = new DutyCycleEncoder(RobotMap.INDEXER_DIO_ENCODER_PORT);
        
        chamberStates = new boolean[5];

        updraw.setNeutralMode(NeutralMode.Coast);

        updraw.config_kP(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_P);
        updraw.config_kI(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_I);
        updraw.config_kD(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_D);
        updraw.config_kF(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_FF);

        updraw.config_kP(UPDRAW_VELOCITY_SLOT, UPDRAW_V_P);
        updraw.config_kI(UPDRAW_VELOCITY_SLOT, UPDRAW_V_I);
        updraw.config_kD(UPDRAW_VELOCITY_SLOT, UPDRAW_V_D);
        updraw.config_kF(UPDRAW_VELOCITY_SLOT, UPDRAW_V_FF);

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);
    }

    public int totalNumOfBalls() {
        int totalNumBalls = 0;
        for (boolean c : chamberStates) {
            if (c) {
                totalNumBalls++;
            }
        }
        return totalNumBalls;
    }

    /**
     * Updates the states of the chambers after intaking a ball, and stops the ToF
     * sensor
     */
    public void intakeBall() {
        if (ballIn()) {
            chamberStates[getBottomChamber()] = true;
            indexerIntakeSensor.stop();
        }
    }

    /**
     * Prepares the indexer for intaking by starting the ToF sensor and moving the
     * indexer in position to intake
     */
    public void prepareToIntake() {
        setTargetAngleInDegrees(INDEXER_DEGREES_PER_CHAMBER * getIntakeChamber() + CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES);
        indexerIntakeSensor.start();
    }

    /**
     * Prepares the indexer to uptake a ball for shooting by rotating to the shoot position
     */
    public void shoot() {
        setTargetAngleInDegrees(getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES);
    }

    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(encoder.getPosition(), setpoint, ALLOWED_ERR));
    }

    /**
     * TODO: Use absolute encoder
     */
    public void zeroRevolver() {
        
    }

    /**
     * Run the uptake, emptying the top chamber of the indexer
     * 
     * @param percentOutput percent output for the updraw
     */
    public void updraw() {
        updraw.set(ControlMode.PercentOutput, UPDRAW_SPEED);
    }

    public void stopUptake() {
        updraw.set(ControlMode.PercentOutput, 0);
    }

    /**
     * 
     * @return true if the updraw is spinning at close to its target speed for
     *         updraw-ing
     */
    public boolean updrawIsUpToSpeed() {
        double c = updraw.getMotorOutputPercent(); // We can't tell if it's actually up to speed, but we're going off of
                                                   // "is it running"
        return MathUtils.doublesEqual(c, UPDRAW_SPEED, 0.05);
    }

    /**
     * Sets the indexer target position to a "staging" position -- 1/2 chamber off
     * -- for holding the ball before the updraw is ready.
     */
    public void rotateToLoadShoot() {
        setTargetAngleInDegrees((getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES)
                - INDEXER_DEGREES_PER_CHAMBER / 2);
    }

    /**
     * Turns to a target angle the most efficient way
     */
    @Override
    public void setTargetAngleInDegrees(double angleDegrees) {
        double currentAngle = getCurrentAngleInDegrees();
        double diff = Math.abs(angleDegrees - currentAngle);
        if (diff > 180) {
            setSmartMotionTarget(getMotorRotationsFromAngle(angleDegrees - 360));
        } else {
            setSmartMotionTarget(getMotorRotationsFromAngle(angleDegrees));
        }
    }

    private int getTopChamber() {
        double pos = getPosition() % 1.0;
        if (pos < 0) {
            pos++;
        }
        // 0.2 - 0.4: 1
        // 0.4 - 0.6: 0
        // 0.6 - 0.8: 4
        // 0.8-1.0: 3
        // 0.0 - 0.2: 2
        return (2 - (int) (pos / 0.2)) % 5;

        /*
        What the expression above does is equivalent to all these if statements below:

        if (0.2 <= pos && 0.4 > pos){
            return 1;
        }
        if (0.4 <= pos && 0.6 > pos){
            return 0;
        }
        if (0.6 <= pos && 0.8 > pos){
            return 4;
        }
        if (0.8 <= pos && 1.0 > pos){
            return 3;
        }
        if (0.0 <= pos && 0.2 > pos){
            return 2;
        }
        */
    }

    private int getBottomChamber() {

        double pos = getPosition() % 1.0;
        if (pos < 0) {
            pos++;
        }
        // 0.9 - 0.1: 0
        // 0.1-0.3: 1
        // 0.3-0.5: 2
        // 0.5-0.7: 3
        // 0.7-0.9: 4
        return (int) (((pos + 0.1) % 1.0) / 0.2);

        /*
        What the expression above does is equivalent to all these if statements below:

        if (0.9 <= pos || 0.1 > pos){
            return 0;
        }
        if (0.1 <= pos && 0.3 > pos){
            return 1;
        }
        if (0.3 <= pos && 0.5 > pos){
            return 2;
        }
        if (0.5 <= pos && 0.7 > pos){
            return 3;
        }
        if (0.7 <= pos && 0.9 > pos){
            return 4;
        }
        */
    }

    // Designed by JOSHIE SANGYALSWEIO
    private int getIntakeChamber() {
        if (totalNumOfBalls() == 5) {
            return -1;
        }
        int currentBottom = getBottomChamber();
        int[] toCheck = { currentBottom, (currentBottom + 1) % 5, (currentBottom - 1) % 5, (currentBottom + 2) % 5,
                (currentBottom - 2) % 5 };
        for (int i = 0; i < toCheck.length; i++) {
            int chamberNum = toCheck[i];
            if (!chamberStates[chamberNum]) {
                return chamberNum;
            }
        }
        return currentBottom;
    }

    private int getShootChamber() {
        if (totalNumOfBalls() == 5) {
            return getTopChamber();
        }
        int currentTop = getTopChamber();
        int[] toCheck = { currentTop, (currentTop + 1) % 5, (currentTop - 1) % 5, (currentTop + 2) % 5,
                (currentTop - 2) % 5 };
        for (int i = 0; i < toCheck.length; i++) {
            int chamberNum = toCheck[i];
            if (chamberStates[chamberNum]) {
                return chamberNum;
            }
        }
        return currentTop;
    }

    /**
     * @return whether a ball has been fully intaked (i.e. is all the way in the
     *         bottom chamber)
     */
    public boolean ballIn() {
        int range = indexerIntakeSensor.getDistance();
        if (range >= TOF_BALL_IN_MIN_RANGE && range <= TOF_BALL_IN_MAX_RANGE) {
            return true;
        }
        return false;
    }

    public double getSpeed() {
        return rotator.get();
    }

    /**
     * @return the position, in number of rotations of the indexer
     */
    public double getPosition() {
        return encoder.getPosition() / ROTATOR_GEAR_RATIO;
    }

    @Override
    public HealthState checkHealth() {
        // if either the rotator or updraw breaks, we can't use the indexer anymore. 
        // Same deal if the indexer is jammed (but that's recoverable).
        if (isSparkMaxErrored(rotator) || isPhoenixControllerErrored(updraw) || indexerIsJammed) {
            return HealthState.RED;
        }
        // if the ToF sensor breaks but nothing else,
        // the next option would be manual control -- not a fatal issue
        if (indexerIntakeSensor == null || !indexerIntakeSensor.isHealthy()) {
            return HealthState.YELLOW;
        }
        return HealthState.GREEN;
    }

    @Override
    public void moveByPercentOutput(double output) {
        rotator.set(output);
    }

    /**
     * @return the angle the indexer is currently turned to, between 0 and 360
     */
    @Override
    public double getCurrentAngleInDegrees() {
        return (getUnadjustedPosition() % this.ROTATOR_GEAR_RATIO) * 360;
    }

    public boolean isIndexerJammed() {
        return this.indexerIsJammed;
    }

    @Override
    public void mustangPeriodic() {
        updrawPreviousCurrent = updrawCurrent;
        updrawCurrent = updraw.getSupplyCurrent();
        double updraw_currentChange = updrawCurrent - updrawPreviousCurrent;

        indexerPreviousCurrent = indexerCurrent;
        indexerCurrent = rotator.getOutputCurrent();

        // Check if indexer is jammed: if the indexer rotator's current stays under the
        // peak allowed (which needs to be defined), then we're good.
        // If the indexer is running at a current above the defined peak for longer than
        // a single instant, then the indexer may be jammed.
        if (indexerCurrent < UPDRAW_PEAK_CURRENT_LIMIT) {
            indexerIsJammed = false;
        } else if (MathUtils.doublesEqual(indexerCurrent, indexerPreviousCurrent, 0.01)) {
            indexerIsJammed = true;
        }

        // Use current to check if a ball has successfully left the indexer through the
        // updraw. If so, marks the top chamber as empty
        if (updraw_currentChange > UPDRAW_SHOOT_CURRENT_CHANGE_THRESHOLD)
            ballIsUpdrawing = true;
        if (ballIsUpdrawing && updraw_currentChange < UPDRAW_SHOOT_COMPLETED_CURRENT_CHANGE) {
            ballIsUpdrawing = false;
            chamberStates[getTopChamber()] = false;
        }
    }

    public boolean isShootingChamberEmpty() {
        return !chamberStates[getTopChamber()];
    }

}