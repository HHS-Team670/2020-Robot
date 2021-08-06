package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.TalonSRXFactory;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the ball indexer subsystem, which tracks and stores up to 3 balls.
 * 
 * @author pallavidas, aadityaraj
 */

// NEW 2021 INDEXER
public class Indexer extends MustangSubsystemBase {

    private Conveyor conveyor;
    //private List<SparkMAXLite> controllers; 
    // private SparkMAXLite frontMotor, backMotor;
    private CANSparkMax frontMotor, backMotor;
    //private CANEncoder frontEncoder, backEncoder;
    private TalonSRX updraw; //motor

    private List<BallSensor> sensors = new ArrayList<BallSensor>();

    //private DutyCycleEncoder revolverAbsoluteEncoder;
    // private Solenoid conveyorToIndexerPusher;

    // private boolean pusherDeployed;

    /*
     * Ranges (in mm) from the TOF sensor for which we know the ball was fully
     * intaked into the bottom chamber.
     */
    // private int TOF_BALL_IN_MIN_RANGE = 20; // from testing 2/16
    // private int TOF_BALL_IN_MAX_RANGE = 50; // from testing 2/16
    // private int TOF_DISTANCE_FOR_PUSH = 110; // from testing 2/19

    private boolean[] chamberStates;
    //TODO for below: should not be arraylist bc then you don't know where the balls are if there are <3 in the indexer
    //j use array, remove chamberStatesList
    private List<Boolean> chamberStatesList; 

    private double updrawCurrent; //electrical current
    // private double updrawPreviousCurrent;

    private int frontExceededCurrentLimitCount = 0;
    private int backExceededCurrentLimitCount = 0;
    private int countToPush = 0;
    private boolean unjamMode = false;
    private boolean indexerIsJammed = false;
    private Long updrawStartTime;
    private double frontSpeed, backSpeed;

    private static final double INDEXER_PEAK_CURRENT = 8; // TODO: find this

    private boolean updrawingMode;
    private boolean isIntaking = true;

    private static final double ABSOLUTE_ENCODER_POSITION_AT_REVOLVER_ZERO = 0.6799; // From 2/17

    private double UPDRAW_SPEED = 0.9;
    private double INDEXER_SPEED = 0.5; // TODO: find this

    private int isUpdrawingCount = 0;
    private int totalNumBalls;

    // TODO: find these values
    private static final double UPDRAW_SHOOT_CURRENT_CHANGE_THRESHOLD = 0;
    private static final double UPDRAW_SHOOT_COMPLETED_CURRENT_CHANGE = 0;

    // Updraw normally at around/below 9A when it's just spinning, peaks when ball
    // touches
    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 9;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 15;

    public Indexer(Conveyor conveyor) {
        super();
        // controllers =
        // SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SPARK_LEFT_MOTOR_1,
        // RobotMap.SPARK_LEFT_MOTOR_2, false, MotorConfig.Motor_Type.NEO);

        frontMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.FRONT_MOTOR, Motor_Type.NEO_550);
        backMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.BACK_MOTOR, Motor_Type.NEO_550);
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();
        frontSpeed = INDEXER_SPEED;
        backSpeed = INDEXER_SPEED * -1; //clockwise vs counterclockwise
        this.conveyor = conveyor;

        // Updraw should be inverted
        this.updraw = TalonSRXFactory.buildFactoryTalonSRX(RobotMap.UPDRAW_SPINNER, false);

        BallSensor entranceSensor = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_1_PORT);
        BallSensor indexerSensorChamber1 = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_2_PORT);
        BallSensor indexerSensorChamber2 = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_3_PORT);
        BallSensor exitSensor = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_4_PORT);

        // TODO: MAP THESE SENSORS
        sensors.add(entranceSensor); // intake to indexer
        sensors.add(indexerSensorChamber1); // chamber 1
        sensors.add(indexerSensorChamber2); // chamber 2
        sensors.add(exitSensor); // exit sensor (indexer to updraw)

        this.revolverAbsoluteEncoder = new DutyCycleEncoder(RobotMap.INDEXER_DIO_ENCODER_PORT);

        chamberStates = new boolean[4];

        updraw.setNeutralMode(NeutralMode.Coast); //free, nothing holding instead of brake

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);

        updraw.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        updraw.enableVoltageCompensation(true);

        SmartDashboard.putNumber("Front Motor Speed", 0.0);
        SmartDashboard.putNumber("Back Motor Speed", 0.0);
    }

    private void pushGameDataToDashboard() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("/SmartDashboard");
        NetworkTableEntry gameData = table.getEntry("Balls");
        gameData.forceSetNumber(totalNumBalls);
    }

    /**
     * At the beginning of autonomous, we preload the indexer with 3 balls. All
     * three are in the indexer, ready to shoot
     */
    public void setChamberStatesForMatchInit() {
        //chamberStatesList.add(true); //would be this 3x, but array instead
        // chamberStates[0] = false;
        // chamberStates[1] = true;
        // chamberStates[2] = true;
        // chamberStates[3] = true;

    }

    // public int totalNumOfBalls() { // TODO fix bug where ball count will repeat
    // when ball is detected by two sensors
    // int totalNumBalls = 0;
    // for (int i = 0; i < sensors.size(); i++) {
    // if (sensors.get(i).getDistance() )
    // if (sensors.get(i).getDistance() < Constants.INDEXER_WIDTH) {
    // totalNumBalls++;
    // }
    // }
    // return totalNumBalls;
    // }
    public void checkBallEntry() {
        // if (!chamberStates && (sensors.get(0).getDistance() == 12.7)) {
        //     totalNumBalls++;
        // }
        if (chamberStates[0] && (sensors.get(0).isBallDetected())) {
            totalNumBalls++;
        }
        // if (sensors.get(0).isBallDetected() && chamberStatesList.size() < 5){
        //     totalNumBalls++;
        //     chamberStatesList.add(true);
        // }
    }

    public void checkBallExit() { //TODO: 
        if (chamberStates[3] && (sensors.get(0).isBallDetected())) {
            totalNumBalls--;
        }
        // if (chamberStatesList.size() == 3 && (sensors.get(0).isBallDetected())) {
        //     totalNumBalls--;
        // }
    }

    public int totalNumBalls() {
        return totalNumBalls;
    }

    public void changeTotalNumBalls(int num) {
        totalNumBalls += num;
    }

    /**
     * Updates the states of the chambers
     */
    public void updateChamberStates() { // change to 'updateChamberStates?'
        for (int i = 0; i < sensors.size(); i++) {
            chamberStates[i] = sensors.get(i).isBallDetected();
            
            // if (sensors.get(i).getDistance() < RobotConstants.SENSOR_TO_BALL) { // TODO make distance to orange belt constant
            //     chamberStates[i] = true;
            //     // latestSensor = i + 1;
            // } else {
            //     chamberStates[i] = false;
            // }
        }
    }

    public void setSpeed(double frontSpeed, double backSpeed) {
        this.frontSpeed = frontSpeed;
        this.backSpeed = backSpeed;
        // SmartDashboard.putNumber("Front Motor Speed", this.frontSpeed);
        // SmartDashboard.putNumber("Back Motor Speed", this.backSpeed);
        run();
    }

    public void run() {
        frontMotor.set(frontSpeed);
        backMotor.set(backSpeed);
    }

    public void stop() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    public boolean isFrontRunning() {
        return frontMotor.get() > 0;
    }

    public boolean isBackRunning() {
        return backMotor.get() > 0;
    }

    // public void toggleUpdraw() {
    //     double c = updraw.getMotorOutputPercent();
    //     if (MathUtils.doublesEqual(c, 0.0, 0.1)) {
    //         updraw(false);
    //     } else {
    //         stopUpdraw();
    //     }
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
        // if (reversed) {
        //     updraw.set(ControlMode.PercentOutput, -0.8 * UPDRAW_SPEED);
        // } else {
        //     updraw.set(ControlMode.PercentOutput, UPDRAW_SPEED);
        // }

        updraw.set(ControlMode.PercentOutput, reversed ? -0.8 * UPDRAW_SPEED : UPDRAW_SPEED);
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
        return System.currentTimeMillis() >= updrawStartTime + 500;

    }

    public int getTopChamber() {
        for (int i = 4; i >= 0; i--) { //TODO: shouldn't it be i = 3 instead of 4?
            if (chamberStates[i])
                return i;
        }

        return -1; // no balls in
    }

    public int getBottomChamber() {
        for (int i = 0; i < 4; i++) {
            if (chamberStates[i])
                return i;
        }

        return -1; // no balls in
    }

    // public boolean chambersEmpty() {
    // for (int i = 0; i < sensors.size(); i++) {
    // if (sensors.get(i).getDistance() <= Constants.INDEXER_WIDTH) { // TODO make
    // distance to orange belt constant
    // return false;
    // }
    // }
    // return true;
    // }

    public double getFrontMotorSpeed() {
        return frontEncoder.getVelocity();
    }

    public double getBackMotorSpeed() {
        return backEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRotations() {
        return ((revolverAbsoluteEncoder.get() + 1.0) % 1.0);
    }

    @Override
    public HealthState checkHealth() {
        // if either the rotator or updraw breaks, we can't use the indexer anymore.
        // Same deal if the indexer is jammed (but that's recoverable).
        // CANError frontError = frontMotor.getLastError();
        // CANError backError = backMotor.getLastError();

        // boolean isFrontError = isSparkMaxErrored(frontMotor);
        // boolean isBackError = isSparkMaxErrored(backMotor);
        // boolean isRotatorError = isSparkMaxErrored(rotator);
        // boolean isUpdrawError = isPhoenixControllerErrored(updraw);
        // if (isUpdrawError || indexerIsJammed || isFrontError || isBackError) {
        // MustangNotifications.reportError("RED Errors: front: %s, back: %s",
        // frontError, backError);
        // isIntaking = false;
        // return HealthState.RED;
        // }
        // if the ToF sensor breaks but nothing else,
        // the next option would be manual control -- not a fatal issue
        // for (int i = 0; i < sensors.size(); i++) {
        // if (sensors.get(i) == null || !sensors.get(i).isHealthy()) {
        // return HealthState.YELLOW;
        // }
        // }
        // isIntaking = true;
        return HealthState.GREEN;
    }

    public void moveByPercentOutput(double output) {
        frontMotor.set(output);
        backMotor.set(output);
    }

    /**
     * 
     * @return whether the rotator current has been
     */

    public boolean frontMotorJammed() {
        double indexerCurrent = frontMotor.getOutputCurrent();
        if (indexerCurrent > 0.2) {
            if (indexerCurrent >= INDEXER_PEAK_CURRENT) {
                frontExceededCurrentLimitCount++;
            } else {
                Logger.consoleLog("Front motor not jammed. current was lower than peak current");
                frontExceededCurrentLimitCount = 0;
            }
            if (frontExceededCurrentLimitCount >= 4) { // 4 consecutive readings higher than peak
                Logger.consoleError("Front motor jammed");
                return true;
            }
        }
        return false;
    }

    public boolean backMotorJammed() {
        double indexerCurrent = backMotor.getOutputCurrent();
        if (indexerCurrent > 0.2) {
            if (indexerCurrent >= INDEXER_PEAK_CURRENT) {
                backExceededCurrentLimitCount++;
            } else {
                Logger.consoleLog("Back motor not jammed. current was lower than peak current");
                backExceededCurrentLimitCount = 0;
            }
            if (backExceededCurrentLimitCount >= 4) { // 4 consecutive readings higher than peak
                Logger.consoleLog("Back motor jammed");
                return true;
            }
        }
        return false;
    }

    public boolean isUpdrawing() {
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
        // unjamMode = true ;
        // posWhenJammed = getCurrentAngleInDegrees();
        // // Move to the previous chamber since most common jam case seems to be on
        // bottom
        // if (setpoint - getMotorRotationsFromAngle(posWhenJammed) > 0) {
        // setTemporaryMotionTarget(setpoint -
        // getMotorRotationsFromAngle(INDEXER_DEGREES_PER_CHAMBER));
        // } else {
        // setTemporaryMotionTarget(setpoint +
        // getMotorRotationsFromAngle(INDEXER_DEGREES_PER_CHAMBER));
        // }
        // timer.reset();
        // clearSetpoint();
        // setRotatorMode(true);

        // }

        // if(unjamMode){
        // if(timer.hasElapsed(2.0)){
        // setRotatorMode(false);
        // Logger.consoleLog("Unjam Timer ended. CurrentMode %s",
        // rotator.getIdleMode());
        // unjamMode = false;
        // }
        // }
        // } else if (unjamMode && MathUtils.doublesEqual(tempSetpoint,
        // rotator_encoder.getPosition(), ALLOWED_ERR)) {
        // // deployPusher(true);
        // // countToPush++;
        // // if (countToPush == 6){
        // // deployPusher(false);
        // // countToPush = 0;
        // // unjamMode = false;
        // isIntaking = true;
        // }
        // Use current to check if a ball has successfully left the indexer through the
        // updraw. If so, marks the top chamber as empty
        setSpeed((SmartDashboard.getNumber("Front Motor Speed", 0.0)),
                (SmartDashboard.getNumber("Back Motor Speed", 0.0)));

        updrawingMode = isUpdrawing();
        if (updrawingMode && !isUpdrawing()) { // We were updrawing but no current spike is detected anymore
            updrawingMode = false;
            chamberStates[3] = false;
        }
        updateChamberStates();
        pushGameDataToDashboard();
    }

    public boolean isChamberFull(int chamber) {
        return chamberStates[chamber];
    }

    public boolean ballInChamber(int chamber) {
        return sensors.get(chamber).getDistance() < RobotConstants.INDEXER_WIDTH;
    }

    public int getGapChamber() {
        for (int i = 1; i < chamberStates.length - 1; i++) {
            if (!chamberStates[i]) {
                if (chamberStates[i - 1] && chamberStates[i + 1]) {
                    return i;
                }
            }
        }
        return -1;
    }

    public boolean motorJammed() {
        return frontMotorJammed() || backMotorJammed();
    }

    public CANSparkMax getFrontMotor() {
        return frontMotor;
    }

    public CANSparkMax getBackMotor() {
        return backMotor;
    }

    public boolean[] getChamberStates() {
        return chamberStates;
    }

    public BallSensor getSensor(int sensor) {
        return sensors.get(sensor);
    }

    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }
}