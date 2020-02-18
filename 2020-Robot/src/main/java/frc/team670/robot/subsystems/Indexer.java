package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private TalonSRX updraw;

    private TimeOfFlightSensor indexerIntakeSensor;
    private DutyCycleEncoder revolverAbsoluteEncoder;

    /*
     * Ranges (in mm) from the TOF sensor for which we know the ball was fully
     * intaked into the bottom chamber.
     */
    private int TOF_BALL_IN_MIN_RANGE = 20; // from testing 2/16
    private int TOF_BALL_IN_MAX_RANGE = 40; // from testing 2/16

    private boolean[] chamberStates;
    private double updrawCurrent;
    private double updrawPreviousCurrent;

    private int exceededCurrentLimitCount = 0;
    private boolean unjamMode = false;
    private boolean indexerIsJammed = false;

    private static final double INDEXER_PEAK_CURRENT = 20; //TODO: find this

    private boolean ballIsUpdrawing;

    private static final double ABSOLUTE_ENCODER_POSITION_AT_REVOLVER_ZERO = 0.6799; //From 2/17

    // For testing purposes
    private double UPDRAW_SPEED = 0.3;

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

        public double getFF() {  // Good enough for 2/16
            return 0.0001;
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
            return 150; //gearing 100 * 1.5 from belt. Changed to this from 35 gearing on 2/18.
        }
        
        public IdleMode setRotatorIdleMode(){
            return IdleMode.kBrake;
        }

    }

    public static final Config INDEXER_CONFIG = new Config();

    public Indexer() {
        super(INDEXER_CONFIG);

        // Updraw should be inverted
        // this.updraw = new TalonSRX(RobotMap.UPDRAW_SPINNER);
        // updraw.setInverted(true);
        this.updraw = TalonSRXFactory.buildFactoryTalonSRX(RobotMap.UPDRAW_SPINNER, true);

        SmartDashboard.putNumber("Indexer Speed", 0.0);
        SmartDashboard.putNumber("Updraw Speed", 0.0);
        SmartDashboard.putNumber("Updraw current", 0.0);
        SmartDashboard.putNumber("Indexer current", 0.0);

        this.indexerIntakeSensor = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_PORT);
        this.revolverAbsoluteEncoder = new DutyCycleEncoder(RobotMap.INDEXER_DIO_ENCODER_PORT);
        this.indexerIntakeSensor.start();
        SmartDashboard.putNumber("ToF Sensor", indexerIntakeSensor.getDistance());

        chamberStates = new boolean[5];

        updraw.setNeutralMode(NeutralMode.Coast);

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);

        updraw.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        updraw.enableVoltageCompensation(true); 
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

    public void rotateByOneChamber(){
        setSystemTargetAngleInDegrees(getCurrentAngleInDegrees() + INDEXER_DEGREES_PER_CHAMBER);
    }

    /**
     * Prepares the indexer for intaking by starting the ToF sensor and moving the
     * indexer in position to intake
     */
    public void prepareToIntake() {
        setSystemTargetAngleInDegrees(INDEXER_DEGREES_PER_CHAMBER * getIntakeChamber() + CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES);
        indexerIntakeSensor.start();
    }

    /**
     * Prepares the indexer to uptake a ball for shooting by rotating to the shoot
     * position
     */
    public void shoot() {
        setSystemTargetAngleInDegrees(getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES);
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
        setSystemTargetAngleInDegrees((getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES)
                - INDEXER_DEGREES_PER_CHAMBER / 2);
    }

    /**
     * Turns to a target angle the most efficient way
     */
    @Override
    public void setSystemTargetAngleInDegrees(double angleDegrees) {
        double currentAngle = getCurrentAngleInDegrees();
        double diff = Math.abs(angleDegrees - currentAngle);
        if (diff > 180) {
            setSystemMotionTarget(getMotorRotationsFromAngle(angleDegrees - 360));
        } else {
            setSystemMotionTarget(getMotorRotationsFromAngle(angleDegrees));
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
        return (7 - (int) (pos / 0.2)) % 5;
        
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

    public double getAbsoluteEncoderRotations(){
        return ((revolverAbsoluteEncoder.get() + 1.0) % 1.0);
    }

    public void setEncoderPositionFromAbsolute(){
        rotator_encoder.setPosition((rotator_encoder.getPosition() - ABSOLUTE_ENCODER_POSITION_AT_REVOLVER_ZERO) % 1.0);
    }

    /**
     * @return the position, in number of rotations of the indexer
     */
    public double getPosition() {
        return rotator_encoder.getPosition() / ROTATOR_GEAR_RATIO;
    }

    public void test() {

        updraw.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Updraw Speed", 0.0));

        SmartDashboard.putNumber("Updraw current", updraw.getStatorCurrent());

        SmartDashboard.putNumber("Indexer current", rotator.getOutputCurrent());

        rotator.set(SmartDashboard.getNumber("Indexer Speed", 0.0));

        SmartDashboard.putNumber("Rotator velocity", rotator.getEncoder().getVelocity());

        SmartDashboard.putNumber("ToF Sensor", indexerIntakeSensor.getDistance());
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

    public boolean isJammed() {
        double indexerCurrent = rotator.getOutputCurrent();
        if (indexerCurrent > 0.2){
            if (indexerCurrent >= INDEXER_PEAK_CURRENT) {
                exceededCurrentLimitCount++;
            } else {
                exceededCurrentLimitCount = 0;
            }
            if (exceededCurrentLimitCount >= 4){ // 4 consecutive readings higher than peak
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
        // Checking both because current readings are really unreliable when not trying to move.
		if (!hasReachedTargetPosition() && isJammed()) {
            unjamMode = true;
            posWhenJammed = getCurrentAngleInDegrees();
            // When unjamming, might be useful to slow down the indexer a bit. 
            // Let's see if we need this. 
            temporaryScaleSmartMotionMaxVelAndAccel(0.75); 
            if (setpoint - posWhenJammed > 0) {
            // TODO find how much we actually want to move it: is half a chamber too much?
                setTemporaryMotionTarget(getMotorRotationsFromAngle(posWhenJammed - INDEXER_DEGREES_PER_CHAMBER / 2));
            } else {
                setTemporaryMotionTarget(getMotorRotationsFromAngle(posWhenJammed + INDEXER_DEGREES_PER_CHAMBER / 2));
            }
        } else if (unjamMode) {
            unjamMode = false;
            resetSmartMotionSettingsToSystem();
            setSystemMotionTarget(setpoint);
        }

        // Use current to check if a ball has successfully left the indexer through the
        // updraw. If so, marks the top chamber as empty
        if (updraw_currentChange > UPDRAW_SHOOT_CURRENT_CHANGE_THRESHOLD){
            ballIsUpdrawing = true;
        }
        if (ballIsUpdrawing && updraw_currentChange < UPDRAW_SHOOT_COMPLETED_CURRENT_CHANGE) {
            ballIsUpdrawing = false;
            chamberStates[getTopChamber()] = false;
        }
    }

    public boolean isShootingChamberEmpty() {
        return !chamberStates[getTopChamber()];
    }

}