package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.team670.mustanglib.dataCollection.sensors.TimeOfFlightSensor;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.TalonSRXFactory;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.MustangNotifications;
import java.util.List;
import java.util.ArrayList;
/**
 * Represents the ball indexer subsystem, which tracks and stores up to 5 balls.
 * 
 * @author ctychen, eddieli, ruchidixit
 */
public class NewIndexer extends MustangSubsystemBase {

    private Conveyor conveyor;
    private List<SparkMAXLite> controllers;
    private SparkMAXLite frontMotor, backMotor;
    private CANEncoder frontEncoder, backEncoder;
    private TalonSRX updraw;

    private TimeOfFlightSensor entranceSensor; //conveyor to indexer sensor (also chamber state 0)
    private TimeOfFlightSensor indexerSensorChamber1;
    private TimeOfFlightSensor indexerSensorChamber2;
    private TimeOfFlightSensor indexerSensorChamber3;
    private TimeOfFlightSensor exitSensor; //indexer to updraw sensor
    private List<TimeOfFlightSensor> sensors = new ArrayList<TimeOfFlightSensor>();
    
    private DutyCycleEncoder revolverAbsoluteEncoder;
    //private Solenoid conveyorToIndexerPusher;

    //private boolean pusherDeployed;

    /*
     * Ranges (in mm) from the TOF sensor for which we know the ball was fully
     * intaked into the bottom chamber.
     */
    private int TOF_BALL_IN_MIN_RANGE = 20; // from testing 2/16
    private int TOF_BALL_IN_MAX_RANGE = 50; // from testing 2/16
    private int TOF_DISTANCE_FOR_PUSH = 110; // from testing 2/19
    private int INDEXXER_WIDTH = 8; // in inches
    
    private boolean[] chamberStates;
    private double updrawCurrent;
    private double updrawPreviousCurrent;

    private int exceededCurrentLimitCount = 0;
    private int countToPush = 0;
    private boolean unjamMode = false;
    private boolean indexerIsJammed = false;
    private Long updrawStartTime;

    private static final double INDEXER_PEAK_CURRENT = 8; // TODO: find this

    private boolean updrawingMode;
    // private boolean isIntaking = false;

    private static final double ABSOLUTE_ENCODER_POSITION_AT_REVOLVER_ZERO = 0.6799; // From 2/17

    private double UPDRAW_SPEED = 0.9;
    private double INDEXER_SPEED = 0.5; //TODO: find this

    private int isUpdrawingCount = 0;

    // TODO: find these values
    private static final double UPDRAW_SHOOT_CURRENT_CHANGE_THRESHOLD = 0;
    private static final double UPDRAW_SHOOT_COMPLETED_CURRENT_CHANGE = 0;

    // Updraw normally at around/below 9A when it's just spinning, peaks when ball touches
    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 9; 
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 15;

    private static final double INDEXER_DEGREES_PER_CHAMBER = 72;

    // Bottom (intake position) is 0, for chamber 0.
    private static final int CHAMBER_0_AT_TOP_POS_IN_DEGREES = 180; // Shooting position for chamber 0
    private static final int CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES = 0; // Intaking position for chamber 0

    private static Timer timer;

    private int topChamber;
    private boolean running;

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

        public double getMaxRotatorRPM() {
            return 2100; // assuming 1 indexer rotation per second
        }

        public double getMinRotatorRPM() {
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
            return IdleMode.kCoast;
        }

    }

    public static final Config INDEXER_CONFIG = new Config();

    public NewIndexer(Conveyor conveyor) {
        super();
        controllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SPARK_LEFT_MOTOR_1, RobotMap.SPARK_LEFT_MOTOR_2,
        false, MotorConfig.Motor_Type.NEO);

        frontMotor = controllers.get(0);
        backMotor = controllers.get(1);
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();

        this.conveyor = conveyor;

        // Updraw should be inverted
        this.updraw = TalonSRXFactory.buildFactoryTalonSRX(RobotMap.UPDRAW_SPINNER, false);

        entranceSensor = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_1_PORT);
        indexerSensorChamber1 = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_2_PORT);
        indexerSensorChamber2 = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_3_PORT);
        indexerSensorChamber3 = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_4_PORT);
        exitSensor = new TimeOfFlightSensor(RobotMap.INDEXER_ToF_SENSOR_5_PORT);

        //TODO: MAP THESE SENSORS
        sensors.set(0, entranceSensor); //intake to indexer
        sensors.set(1, indexerSensorChamber1); //chamber 1  
        sensors.set(2, indexerSensorChamber2); //chamber 2
        sensors.set(3, indexerSensorChamber3); //chamber 3
        sensors.set(4, exitSensor); //exit sensor (indexer to updraw)

        this.revolverAbsoluteEncoder = new DutyCycleEncoder(RobotMap.INDEXER_DIO_ENCODER_PORT);

        // this.conveyorToIndexerPusher = pusher;
        // this.pusherDeployed = false;

        

        chamberStates = new boolean[4];

        updraw.setNeutralMode(NeutralMode.Coast);

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);

        updraw.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        updraw.enableVoltageCompensation(true);

        //reset();

        timer = new Timer();
        timer.start();
    }

    private void pushGameDataToDashboard(){
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("/SmartDashboard");
        NetworkTableEntry gameData = table.getEntry("Balls");
        gameData.forceSetNumber(totalNumOfBalls());
    }

    /**
     * At the beginning of autonomous, we preload the indexer with 3 balls. 
     * All three are in the indexer, ready to shoot
     */
    public void setChamberStatesForMatchInit(){
        chamberStates[0] = false;
        chamberStates[1] = true;
        chamberStates[2] = true;
        chamberStates[3] = true;
    }

    /**
     * Sets properties to either default or whatever was last recorded, so if we restart/are interrupted we can start back
     */
    // public void reset(){
    //     // this.pusherDeployed = conveyorToIndexerPusher.get();
    //     this.isIntaking = false;
    // }

    // public void deployPusher(boolean toPush) {
    //     this.pusherDeployed = toPush;
    //     this.conveyorToIndexerPusher.set(pusherDeployed);
    //     // if (toPush){
    //     //    conveyor.stop();
    //     // }
    // }

    // public boolean isPusherDeployed(){
    //     return pusherDeployed;
    // }

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
     * Updates the states of the chambers after intaking a ball
     */
    public void intakeBall() { // change to 'updateChamberStates?'
        for (int i = 0; i < sensors.size(); i++) {
            if (sensors.get(i).getDistance() < RobotConstants.INDEXER_WIDTH) { //TODO make indexer width constant
                chamberStates[i] = true;
                // latestSensor = i + 1;
            } else {
                chamberStates[i] = false;
            }
        }
        // if (isIntaking){
        //     IntakingState intakingState = ballIn();
        //     if (intakingState == IntakingState.MAYBE_IN){
        //         deployPusher(true);
        //     }
        //     else if (intakingState == IntakingState.IN){
        //         chamberStates[getBottomChamber()] = true;
        //         deployPusher(false);        
        //         return true;
        //     }
        // }
        // else {
        //     deployPusher(false);
        // }
        // return false;

        
    }

    /**
     * Sets the indexer to move the balls up one chamber.
     */
    public void nextChamber() {

        topChamber = getTopChamber();
        while ((topChamber != 4) && (topChamber != getTopChamber() - 1)) {
            move();
        }
        stop();
        // if (!running) {
        //     move();
        // }
    }

    public void move() {
        frontMotor.set(INDEXER_SPEED);
        backMotor.set(INDEXER_SPEED);
        // running = true;
    }

    public void stop() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
        // running = false;
    }

    // /**
    //  * Sets the indexer to move to the previous chamber.
    //  */
    // public void rotateToPreviousChamber() {
    //     setSystemTargetAngleInDegrees(((getBottomChamber() -1) % 5) * INDEXER_DEGREES_PER_CHAMBER);
    // }

    // /**
    //  * Rotates such that the next empty chamber is at the bottom 
    //  */
    // private void rotateToNextEmptyChamber() {
    //     double setpoint = INDEXER_DEGREES_PER_CHAMBER * getIntakeChamber() + CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES;
    //     setSystemTargetAngleInDegrees(setpoint);
    // }

    // /**
    //  * Rotates such that the chamber at the top of the indexer is empty
    //  */
    // public void rotateToEmptyAtTop()   {
    //     double setpoint = INDEXER_DEGREES_PER_CHAMBER * getIntakeChamber() + CHAMBER_0_AT_TOP_POS_IN_DEGREES;
    //     setSystemTargetAngleInDegrees(setpoint);
    // }

    // public void spinRevolver()  {
    //     Logger.consoleLog("Indexer system current angle degrees %s", getCurrentAngleInDegrees());
    //     Logger.consoleLog("Indexer system current position %s", rotator_encoder.getPosition());
    //     deployPusher(false);
    //     setSystemMotionTarget(getMotorRotationsFromAngle(getCurrentAngleInDegrees() + 360));
    //     Logger.consoleLog("Indexer system set to angle %s", getCurrentAngleInDegrees() + 360);
    //     Logger.consoleLog("Indexer system rotations set to %s", setpoint);
    // }

    /**
     * Prepares the indexer for intaking by starting the ToF sensor and moving the
     * indexer in position to intake
     */
    // public void prepareToIntake() {
    //     isIntaking = true;
    //     // rotateToNextEmptyChamber();
    // }

    /**
     * Prepares the indexer to uptake a ball for shooting by rotating to the shoot
     * position
     */
    public void shoot() {
        // setSystemTargetAngleInDegrees(
        //         getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES);
    }

    public void toggleUpdraw() {
        double c = updraw.getMotorOutputPercent();
        if (MathUtils.doublesEqual(c, 0.0, 0.1)) {
            updraw(false);
        } else {
            stopUpdraw();
        }
    }

    // public boolean hasReachedTargetPosition() {
    //     boolean hasReachedTarget = (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint, ALLOWED_ERR));
    //     if(hasReachedTarget && !isIntaking){
    //         deployPusher(true);
    //     }
    //     return hasReachedTarget;
    // }

    /**
     * Run the uptake, emptying the top chamber of the indexer
     * 
     * @param reversed true to run updraw backwards, false to run normally
     */
    public void updraw(boolean reversed) {
        if (updrawStartTime == null) {
            updrawStartTime = System.currentTimeMillis();
        }
        if (reversed){
            updraw.set(ControlMode.PercentOutput, -0.8 * UPDRAW_SPEED);
        }
        else {
            updraw.set(ControlMode.PercentOutput, UPDRAW_SPEED);
        }
    }

    public void stopUpdraw() {
        updraw.set(ControlMode.PercentOutput, 0);
        updrawStartTime = null;
    }

    /**
     * 
     * @return true if the updraw is spinning at close to its target speed for
     *         updraw-ing
     */
    public boolean updrawIsUpToSpeed() {
        double c = updraw.getMotorOutputPercent(); // We can't tell if it's actually up to speed, but we're going off of
                                                   // "is it running"
        if (updrawStartTime == null) {
            return MathUtils.doublesEqual(c, UPDRAW_SPEED, 0.05);
        }
        if (System.currentTimeMillis() >= updrawStartTime + 500) {
            return true;
        }
        return false;

    }

    // /**
    //  * Sets the indexer target position to a "staging" position -- 1/2 chamber off
    //  * -- for holding the ball before the updraw is ready.
    //  */
    // public void rotateToLoadShoot() {
    //     setSystemTargetAngleInDegrees(
    //             (getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_TOP_POS_IN_DEGREES)
    //                     - INDEXER_DEGREES_PER_CHAMBER / 2);
    // }

    // /**
    //  * Turns to a target angle the most efficient way.
    //  */
    // @Override
    // public void setSystemTargetAngleInDegrees(double angleDegrees) {
    //     //deployPusher(false); //No pusher anymore
    //     double currentAngle = getCurrentAngleInDegrees();
    //     // We find the difference between where we currently are, and where we want to be
    //     int sign = (angleDegrees -currentAngle >= 0 && angleDegrees - currentAngle <= 180) || (angleDegrees-currentAngle <=-180 && angleDegrees-currentAngle>=-360) ? 1 : -1;
    //     Logger.consoleLog("Angle %s, Sign %s, CurrentAngle %s", angleDegrees, sign, currentAngle);
    //     double diff = Math.abs(angleDegrees - currentAngle) % 360;
    //     // If the difference is above 180 degrees, it's better to turn in the opposite direction.
    //     // Otherwise, we continue turning in the same direction.
    //     double finDiff = diff > 180 ? 360 - diff : diff;
    //     // Setpoints are absolute, so we want to move from where we currently are plus the difference.
    //     setSystemMotionTarget(getMotorRotationsFromAngle(currentAngle + finDiff));
    // }

    private int getTopChamber() {
        for (int i = 4; i >= 0; i--) {
            if (chamberStates[i]) 
                return i;
        }

        return -1; //no balls in
    }


    private 
    // private int getBottomChamber() {

    //     double pos = getPosition() % 1.0;
    //     if (pos < 0) {
    //         pos++;
    //     }

    //     // 0.9 - 0.1: 0
    //     // 0.1-0.3: 1
    //     // 0.3-0.5: 2
    //     // 0.5-0.7: 3
    //     // 0.7-0.9: 4

    //     return (int) (((pos + 0.1) % 1.0) / 0.2);

    //     /*
    //      * What the expression above does is equivalent to all these if statements
    //      * below: 
    //      * if (0.9 <= pos || 0.1 > pos) { 
    //      * return 0; 
    //      * } 
    //      * if (0.1 <= pos && 0.3 > pos) { 
    //      * return 1; 
    //      * } 
    //      * if (0.3 <= pos && 0.5 > pos) { 
    //      * return 2; 
    //      * } 
    //      * if (0.5 <= pos && 0.7 > pos) { 
    //      * return 3; 
    //      * } 
    //      * if (0.7 <= pos && 0.9 > pos) { 
    //      * return 4; 
    //      * }
    //      */
    // }

    // // Designed by JOSHIE SANGYALSWEIO
    // private int getIntakeChamber() {
    //     if (totalNumOfBalls() == 5) {
    //         return -1;
    //     }
    //     int currentBottom = getBottomChamber();
    //     int[] toCheck = { currentBottom, (currentBottom + 1) % 5, (currentBottom + 4) % 5, (currentBottom + 2) % 5,
    //             (currentBottom + 3) % 5 };
    //     for (int i = 0; i < toCheck.length; i++) {
    //         int chamberNum = toCheck[i];
    //         if (!chamberStates[chamberNum]) {
    //             return chamberNum;
    //         }
    //     }
    //     return currentBottom;
    // }

    // private int getShootChamber() {
    //     if (totalNumOfBalls() == 5) {
    //         return getTopChamber();
    //     }
    //     int currentTop = getTopChamber();
    //     int[] toCheck = { currentTop, (currentTop + 1) % 5, (currentTop + 4 ) % 5, (currentTop + 2) % 5,
    //             (currentTop + 3) % 5 };
    //     for (int i = 0; i < toCheck.length; i++) {
    //         int chamberNum = toCheck[i];
    //         if (chamberStates[chamberNum]) {
    //             return chamberNum;
    //         }
    //     }
    //     return currentTop;
    // }

    /**
     * @return whether a ball has been fully intaked (i.e. is all the way in the
     *         bottom chamber)
     */
    public IntakingState ballIn() {
        
        // int range = indexerIntakeSensor.getDistance();
        // // We shouldn't be detecting "intaked" if the chamber is already full before,
        // // or if we just saw an arm move by
        // if (!chamberStates[getBottomChamber()] && hasReachedTargetPosition()) {
        //     conveyor.run(false);
        //     if (range >= TOF_BALL_IN_MIN_RANGE) {
        //         if (range <= TOF_BALL_IN_MAX_RANGE) {
        //             return IntakingState.IN;
        //         }
        //         if (range <= TOF_DISTANCE_FOR_PUSH) {
        //             return IntakingState.MAYBE_IN;
        //         }
        //     }
        // }
        // return IntakingState.NOT_IN;
    }

    // public double getSpeed() {
    //     return rotator.get();
    // }

    public double getFrontMotorSpeed(){
        return frontEncoder.getVelocity();
    }

    public double getBackMotorSpeed(){
        return backEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRotations() {
        return ((revolverAbsoluteEncoder.get() + 1.0) % 1.0);
    }

    /**
     * Sets the rotator encoder's reference position to the constant obtained from
     * the absolute encoder corresponding to that position.
     */
    // public void setEncoderPositionFromAbsolute() {
    //     clearSetpoint();
    //     rotator_encoder.setPosition(
    //             (getAbsoluteEncoderRotations() - ABSOLUTE_ENCODER_POSITION_AT_REVOLVER_ZERO) * this.ROTATOR_GEAR_RATIO);
    //     Logger.consoleLog("Encoder position set: %s", rotator_encoder.getPosition());
    // }

    /**
     * @return the position, in number of rotations of the indexer
     */
    // public double getPosition() {
    //     return rotator_encoder.getPosition() / ROTATOR_GEAR_RATIO;
    // }

    @Override
    public HealthState checkHealth() {
        // if either the rotator or updraw breaks, we can't use the indexer anymore.
        // Same deal if the indexer is jammed (but that's recoverable).
        CANError frontError = frontMotor.getLastError();
        CANError backError = backMotor.getLastError();

        boolean isFrontError = isSparkMaxErrored(frontMotor);
        boolean isBackError = isSparkMaxErrored(backMotor);
        // boolean isRotatorError = isSparkMaxErrored(rotator);
        boolean isUpdrawError = isPhoenixControllerErrored(updraw);
        if (isUpdrawError || indexerIsJammed || isFrontError) {
            return HealthState.RED;
            MustangNotifications.reportError("RED Errors: front: %s, back: %s", frontError, backError,);
        }
        // if the ToF sensor breaks but nothing else,
        // the next option would be manual control -- not a fatal issue
    //     private TimeOfFlightSensor entranceSensor; //conveyor to indexer sensor (also chamber state 0)
    // private TimeOfFlightSensor indexerSensorChamber1;
    // private TimeOfFlightSensor indexerSensorChamber2;
    // private TimeOfFlightSensor indexerSensorChamber3;
    // private TimeOfFlightSensor exitSensor; //indexer to updraw sensor
        for (int i = 0; i < sensors.size(); i++) {
            if (sensors.get(i) == null || !sensors.get(i).isHealthy() || isBackError) {
                return HealthState.YELLOW;
            }
        }
        // if (indexerIntakeSensor == null || !indexerIntakeSensor.isHealthy() || isBackError) {
        //     return HealthState.YELLOW;
        // }
        return HealthState.GREEN;
    }

    public void moveByPercentOutput(double output) {
        // rotator.set(output);
        frontMotor.set(output);
        backMotor.set(output);
    }

    // /**
    //  * @return the angle the indexer is currently turned to, between 0 and 360
    //  */
    // public double getCurrentAngleInDegrees() {
    //     // find how many rotations the motor has moved compared to the number needed for
    //     // 1 full rotation
    //     // then finds what angle that is
    //     double degrees = (((getUnadjustedPosition() % this.ROTATOR_GEAR_RATIO) / this.ROTATOR_GEAR_RATIO) * 360);
    //     return degrees;
    // }

    /**
     * 
     * @return whether the rotator current has been 
     */
    public boolean frontMotorJammed() {
        double indexerCurrent = frontMotor.getOutputCurrent();
        if (indexerCurrent > 0.2) {
            if (indexerCurrent >= INDEXER_PEAK_CURRENT) {
                exceededCurrentLimitCount++;
            } else {
                Logger.consoleLog("Jammed false. current was lower than peak current");
                exceededCurrentLimitCount = 0;
            }
            if (exceededCurrentLimitCount >= 4) { // 4 consecutive readings higher than peak
                Logger.consoleLog("Jammed true");
                return true;
            }
        }
        return false;
    }

    public boolean isUpdrawing(){
        double updrawCurrent = updraw.getSupplyCurrent();
        if (updrawCurrent > 0.2) {
            if (updrawCurrent >= UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT) {
                isUpdrawingCount++;
            } else {
                isUpdrawingCount = 0;
            }
            if (isUpdrawingCount >= 3) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void mustangPeriodic() {
        double posWhenJammed = 0;
        // If we're trying to move somewhere and it's jammed, we try unjamming it.
        // Checking both because current readings are really unreliable when not trying
        // to move.
        // if (!hasReachedTargetPosition() && isJammed()) {
        //     unjamMode = true ;
            // posWhenJammed = getCurrentAngleInDegrees();
            // // Move to the previous chamber since most common jam case seems to be on bottom
            // if (setpoint - getMotorRotationsFromAngle(posWhenJammed) > 0) {
            //     setTemporaryMotionTarget(setpoint - getMotorRotationsFromAngle(INDEXER_DEGREES_PER_CHAMBER));
            // } else {
            //     setTemporaryMotionTarget(setpoint + getMotorRotationsFromAngle(INDEXER_DEGREES_PER_CHAMBER));
            // }
        //     timer.reset();
        //     clearSetpoint();
        //     setRotatorMode(true);

        // }
        
        // if(unjamMode){
        //     if(timer.hasElapsed(2.0)){
        //         setRotatorMode(false);
        //         Logger.consoleLog("Unjam Timer ended. CurrentMode %s", rotator.getIdleMode());
        //         unjamMode = false;
        //     }
        // }
        // } else if (unjamMode && MathUtils.doublesEqual(tempSetpoint, rotator_encoder.getPosition(), ALLOWED_ERR)) {
        //     // deployPusher(true);
        //     // countToPush++;
        //     // if (countToPush == 6){
        //     //     deployPusher(false);
        //     //     countToPush = 0;
        //     //     unjamMode = false;
        //     isIntaking = true;
        // }

        // if (running && (topChamber + 1 == getTopChamber())) {
        //     stop();
        //     running = false;
        // } 
        // Use current to check if a ball has successfully left the indexer through the
        // updraw. If so, marks the top chamber as empty
        if (isUpdrawing()) {
            updrawingMode = true;
        }
        if (updrawingMode && !isUpdrawing()) { // We were updrawing but no current spike is detected anymore
            updrawingMode = false;
            chamberStates[3] = false;
        }
        // if (isIntaking && intakeBall()) {
        //     unjamMode = false;
        //     rotateToNextEmptyChamber();
        // }
        intakeBall();
        pushGameDataToDashboard();
    }

    // public void stopIntaking() {
    //     isIntaking = false;
    //     // frontMotor.set(0);
    //     // backMotor.set(0);
    //     // if(hasReachedTargetPosition()){
    //     //     deployPusher(true); No pusher anymore
    //     // }
    // }

    public boolean isShootingChamberEmpty() {
        return chamberStates[3] == false;
    }

    /**
     * @param true to set indexer to coast mode, false to set it to brake mode
     */
    // public void setRotatorMode(boolean coast){
    //     if (coast){
    //         rotator.setIdleMode(IdleMode.kCoast);
    //     } else {
    //         rotator.setIdleMode(IdleMode.kBrake);
    //     }
    // }

    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }
}