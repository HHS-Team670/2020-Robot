package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Solenoid;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.TimeOfFlightSensor;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.motorcontroller.MotorConfig;
import frc.team670.robot.utils.motorcontroller.TalonSRXFactory;

/**
 * Represents the ball indexer subsystem, which tracks and stores up to 5 balls.
 * 
 * @author ctychen, eddieli, ruchidixit
 */
public class Indexer extends SparkMaxRotatingSubsystem {

    private Conveyor conveyor;

    private TalonSRX updraw;

    private TimeOfFlightSensor indexerIntakeSensor;
    private DutyCycleEncoder revolverAbsoluteEncoder;
    private Solenoid conveyorToIndexerPusher;

    private boolean pusherDeployed;

    /*
     * Ranges (in mm) from the TOF sensor for which we know the ball was fully
     * intaked into the bottom chamber.
     */
    private int TOF_BALL_IN_MIN_RANGE = 20; // from testing 2/16
    private int TOF_BALL_IN_MAX_RANGE = 50; // from testing 2/16
    private int TOF_DISTANCE_FOR_PUSH = 110; // from testing 2/19

    private boolean[] chamberStates;
    private double updrawCurrent;
    private double updrawPreviousCurrent;

    private int exceededCurrentLimitCount = 0;
    private boolean unjamMode = false;
    private boolean indexerIsJammed = false;

    private static final double INDEXER_PEAK_CURRENT = 12; // TODO: find this

    private boolean ballIsUpdrawing;
    private boolean isIntaking = false;

    private static final double ABSOLUTE_ENCODER_POSITION_AT_REVOLVER_ZERO = 0.6799; // From 2/17

    // For testing purposes
    private double UPDRAW_SPEED = 0.9;

    // TODO: find these values
    private static final double UPDRAW_SHOOT_CURRENT_CHANGE_THRESHOLD = 0;
    private static final double UPDRAW_SHOOT_COMPLETED_CURRENT_CHANGE = 0;

    // TODO: Set these
    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 5;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 15;

    private static final double INDEXER_DEGREES_PER_CHAMBER = 72;

    // Bottom (intake position) is 0, for chamber 0.
    private static final int CHAMBER_0_AT_TOP_POS_IN_DEGREES = 180; // Shooting position for chamber 0
    private static final int CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES = 0; // Intaking position for chamber 0

    private enum IntakingState {
        IN, MAYBE_IN, NOT_IN;
    }

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
            return 0.00001; // Good enough for 2/16
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
        }

        public double getFF() { // Good enough for 2/16
            return 0.0001 * 3; // Gearing was adjusted
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

        public double getRotatorGearRatio() {
            return 150; // gearing 100 * 1.5 from belt. Changed to this from 35 gearing on 2/18.
        }

        public IdleMode setRotatorIdleMode() {
            return IdleMode.kBrake;
        }

    }

    public static final Config INDEXER_CONFIG = new Config();

    public Indexer(Conveyor conveyor) {
        super(INDEXER_CONFIG);

        this.conveyor = conveyor;

        // Updraw should be inverted
        this.updraw = TalonSRXFactory.buildFactoryTalonSRX(RobotMap.UPDRAW_SPINNER, false);

        this.indexerIntakeSensor = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_PORT);
        this.revolverAbsoluteEncoder = new DutyCycleEncoder(RobotMap.INDEXER_DIO_ENCODER_PORT);

        this.conveyorToIndexerPusher = new Solenoid(RobotMap.PCMODULE, RobotMap.BALL_INTO_INDEXER_PUSHER);
        this.pusherDeployed = false;

        chamberStates = new boolean[5];

        updraw.setNeutralMode(NeutralMode.Coast);

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);

        updraw.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        updraw.enableVoltageCompensation(true);

        reset();
    }

    /**
     * At the beginning of autonomous, we preload the indexer with 3 balls, 
     * in the green-labeled chamber (#0) and the chambers on either side of it (#1 and #4).
     * This sets the states of those chambers so we know that those are filled.
     */
    public void setChamberStatesForMatchInit(){
        chamberStates[0] = true;
        chamberStates[1] = true;
        chamberStates[4] = true;
    }

    /**
     * Sets properties to either default or whatever was last recorded, so if we restart/are interrupted we can start back
     */
    public void reset(){
        this.pusherDeployed = conveyorToIndexerPusher.get();
        this.isIntaking = false;
    }

    public void deployPusher(boolean toPush) {
        this.pusherDeployed = toPush;
        this.conveyorToIndexerPusher.set(pusherDeployed);
        if (toPush){
            conveyor.stop();
        }
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
    public boolean intakeBall() {
        if (isIntaking){
            IntakingState intakingState = ballIn();
            if (intakingState == IntakingState.MAYBE_IN){
                deployPusher(true);
            }
            else if (intakingState == IntakingState.IN){
                chamberStates[getBottomChamber()] = true;
                deployPusher(false);        
                return true;
            }
        }
        else {
            deployPusher(false);
        }
        return false;
    }

    /**
     * Sets the indexer to move to the next chamber.
     */
    public void rotateToNextChamber() {
        setSystemTargetAngleInDegrees(((getBottomChamber() + 1) % 5) * INDEXER_DEGREES_PER_CHAMBER);
    }

    private void rotateToNextEmptyChamber() {
        double setpoint = INDEXER_DEGREES_PER_CHAMBER * getIntakeChamber() + CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES;
        setSystemTargetAngleInDegrees(setpoint);
    }

    /**
     * Prepares the indexer for intaking by starting the ToF sensor and moving the
     * indexer in position to intake
     */
    public void prepareToIntake() {
        isIntaking = true;
        rotateToNextEmptyChamber();
    }

    /**
     * Prepares the indexer to uptake a ball for shooting by rotating to the shoot
     * position
     */
    public void shoot() {
        setSystemTargetAngleInDegrees(
                getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES);
    }

    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint, ALLOWED_ERR));
    }

    /**
     * Run the uptake, emptying the top chamber of the indexer
     * 
     */
    public void updraw() {
        updraw.set(ControlMode.PercentOutput, UPDRAW_SPEED);
    }

    public void stopUpdraw() {
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
        setSystemTargetAngleInDegrees(
                (getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES)
                        - INDEXER_DEGREES_PER_CHAMBER / 2);
    }

    /**
     * Turns to a target angle the most efficient way.
     */
    @Override
    public void setSystemTargetAngleInDegrees(double angleDegrees) {
        deployPusher(false);
        double currentAngle = getCurrentAngleInDegrees();
        // We find the difference between where we currently are, and where we want to be
        double diff = Math.abs(angleDegrees - currentAngle) % 360;
        // If the difference is above 180 degrees, it's better to turn in the opposite direction.
        // Otherwise, we continue turning in the same direction.
        double finDiff = diff > 180 ? 360 - diff : diff;
        // Setpoints are absolute, so we want to move from where we currently are plus the difference.
        setSystemMotionTarget(getMotorRotationsFromAngle(currentAngle + finDiff));
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

        return (7 - (int) (pos / 0.2)) % 5;

        /*
         * What the expression above does is equivalent to all these if statements
         * below: 
         * if (0.2 <= pos && 0.4 > pos) {
         *  return 1; 
         * } 
         * if (0.4 <= pos && 0.6 > pos) { 
         * return 0; 
         * } 
         * if (0.6 <= pos && 0.8 > pos) { 
         * return 4; 
         * } 
         * if (0.8 <= pos && 1.0 > pos) { 
         * return 3; 
         * } 
         * if (0.0 <= pos && 0.2 > pos) { 
         * return 2; 
         * }
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
         * What the expression above does is equivalent to all these if statements
         * below: 
         * if (0.9 <= pos || 0.1 > pos) { 
         * return 0; 
         * } 
         * if (0.1 <= pos && 0.3 > pos) { 
         * return 1; 
         * } 
         * if (0.3 <= pos && 0.5 > pos) { 
         * return 2; 
         * } 
         * if (0.5 <= pos && 0.7 > pos) { 
         * return 3; 
         * } 
         * if (0.7 <= pos && 0.9 > pos) { 
         * return 4; 
         * }
         */
    }

    // Designed by JOSHIE SANGYALSWEIO
    private int getIntakeChamber() {
        if (totalNumOfBalls() == 5) {
            return -1;
        }
        int currentBottom = getBottomChamber();
        int[] toCheck = { currentBottom, (currentBottom + 1) % 5, (currentBottom + 4) % 5, (currentBottom + 2) % 5,
                (currentBottom + 3) % 5 };
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
        int[] toCheck = { currentTop, (currentTop + 1) % 5, (currentTop + 4 ) % 5, (currentTop + 2) % 5,
                (currentTop + 3) % 5 };
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
    public IntakingState ballIn() {
        int range = indexerIntakeSensor.getDistance();
        // We shouldn't be detecting "intaked" if the chamber is already full before,
        // or if we just saw an arm move by
        if (!chamberStates[getBottomChamber()] && hasReachedTargetPosition()) {
            conveyor.run(false);
            if (range >= TOF_BALL_IN_MIN_RANGE) {
                if (range <= TOF_BALL_IN_MAX_RANGE) {
                    return IntakingState.IN;
                }
                if (range <= TOF_DISTANCE_FOR_PUSH) {
                    return IntakingState.MAYBE_IN;
                }
            }
        }
        return IntakingState.NOT_IN;
    }

    public double getSpeed() {
        return rotator.get();
    }

    public double getAbsoluteEncoderRotations() {
        return ((revolverAbsoluteEncoder.get() + 1.0) % 1.0);
    }

    /**
     * Sets the rotator encoder's reference position to the constant obtained from
     * the absolute encoder corresponding to that position.
     */
    public void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        rotator_encoder.setPosition(
                (getAbsoluteEncoderRotations() - ABSOLUTE_ENCODER_POSITION_AT_REVOLVER_ZERO) * this.ROTATOR_GEAR_RATIO);
        Logger.consoleLog("Encoder position set: %s", rotator_encoder.getPosition());
    }

    /**
     * @return the position, in number of rotations of the indexer
     */
    public double getPosition() {
        return rotator_encoder.getPosition() / ROTATOR_GEAR_RATIO;
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
        // find how many rotations the motor has moved compared to the number needed for
        // 1 full rotation
        // then finds what angle that is
        double degrees = ((getUnadjustedPosition() % this.ROTATOR_GEAR_RATIO) / this.ROTATOR_GEAR_RATIO) * 360;
        return degrees;
    }

    /**
     * 
     * @return whether the rotator current has been 
     */
    public boolean isJammed() {
        double indexerCurrent = rotator.getOutputCurrent();
        if (indexerCurrent > 0.2) {
            if (indexerCurrent >= INDEXER_PEAK_CURRENT) {
                exceededCurrentLimitCount++;
            } else {
                exceededCurrentLimitCount = 0;
            }
            if (exceededCurrentLimitCount >= 4) { // 4 consecutive readings higher than peak
                return true;
            }
        }
        return false;
    }

    @Override
    public void mustangPeriodic() {
        updrawPreviousCurrent = updrawCurrent;
        updrawCurrent = updraw.getSupplyCurrent();
        double updraw_currentChange = updrawCurrent - updrawPreviousCurrent;
        double posWhenJammed = 0;
        // If we're trying to move somewhere and it's jammed, we try unjamming it.
        // Checking both because current readings are really unreliable when not trying
        // to move.
        if (!hasReachedTargetPosition() && isJammed()) {
            unjamMode = true;
            posWhenJammed = getCurrentAngleInDegrees();
            if (setpoint - getMotorRotationsFromAngle(posWhenJammed) > 0) {
                setTemporaryMotionTarget(getMotorRotationsFromAngle(posWhenJammed - (INDEXER_DEGREES_PER_CHAMBER / 2)));
            } else {
                setTemporaryMotionTarget(getMotorRotationsFromAngle(posWhenJammed + (INDEXER_DEGREES_PER_CHAMBER / 2)));
            }
        } else if (unjamMode && MathUtils.doublesEqual(tempSetpoint, rotator_encoder.getPosition(), ALLOWED_ERR)) {
            unjamMode = false;
            resetSmartMotionSettingsToSystem();
            setSystemMotionTarget(setpoint);
        }

        // Use current to check if a ball has successfully left the indexer through the
        // updraw. If so, marks the top chamber as empty
        if (updraw_currentChange > UPDRAW_SHOOT_CURRENT_CHANGE_THRESHOLD) {
            ballIsUpdrawing = true;
        }
        if (ballIsUpdrawing && updraw_currentChange < UPDRAW_SHOOT_COMPLETED_CURRENT_CHANGE) {
            ballIsUpdrawing = false;
            chamberStates[getTopChamber()] = false;
        }
        if (isIntaking && intakeBall() && totalNumOfBalls() < 5) {
            rotateToNextChamber();
        }
    }

    public void stopIntaking() {
        isIntaking = false;
        deployPusher(true);
    }

    public boolean isShootingChamberEmpty() {
        return !chamberStates[getTopChamber()];
    }
}