package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangNotifications;
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

    private Conveyor conveyor; // we don't actually use this; can remove it from construc
    private CANSparkMax frontMotor, backMotor;
    private CANEncoder frontEncoder, backEncoder;
    private TalonSRX updraw; // motor
    private List<BallSensor> sensors = new ArrayList<BallSensor>();

    private boolean[] chamberStates;

    private int frontExceededCurrentLimitCount = 0;
    private int backExceededCurrentLimitCount = 0;

    private Long updrawStartTime;
    private double frontSpeed, backSpeed, updrawSpeed;

    private boolean updrawingMode;

    private double UPDRAW_SPEED = -0.9;
    private double INDEXER_SPEED = 0.9;

    private int isUpdrawingCount = 0;
    private int totalNumBalls;

    // Updraw normally at around/below 9A when it's just spinning, peaks when ball
    // touches
    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 9;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 15;
    public BallSensor entranceSensor;

    public Indexer(Conveyor conveyor) {
        super();

        frontMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.FRONT_MOTOR, Motor_Type.NEO_550);
        backMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.BACK_MOTOR, Motor_Type.NEO_550);
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();
        frontSpeed = INDEXER_SPEED;
        backSpeed = -1 * INDEXER_SPEED; // clockwise vs counterclockwise
        this.conveyor = conveyor;

        // Updraw should be inverted
        this.updraw = TalonSRXFactory.buildFactoryTalonSRX(RobotMap.UPDRAW_SPINNER, false);

        entranceSensor = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_1_PORT);
        // BallSensor indexerSensorChamber1 = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_2_PORT);
        // BallSensor indexerSensorChamber2 = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_3_PORT);
        // BallSensor exitSensor = new BallSensor(RobotMap.INDEXER_ToF_SENSOR_4_PORT);

        // sensors.add(entranceSensor); // intake to indexer
        // sensors.add(indexerSensorChamber1); // chamber 1
        // sensors.add(indexerSensorChamber2); // chamber 2
        // sensors.add(exitSensor); // exit sensor (indexer to updraw)

        chamberStates = new boolean[4];

        updraw.setNeutralMode(NeutralMode.Coast); // free, nothing holding instead of brake

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);

        updraw.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        updraw.enableVoltageCompensation(true);

        SmartDashboard.putNumber("Front Motor Speed", 0.0);
        SmartDashboard.putNumber("Back Motor Speed", 0.0);
        SmartDashboard.putNumber("Updraw Speed", 0.0);
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
        chamberStates[0] = false;
        chamberStates[1] = true;
        chamberStates[2] = true;
        chamberStates[3] = true;
    }

    /**
     * Updates the states of the chambers
     */
    public void updateChamberStates() {
        for (int i = 0; i < sensors.size(); i++) {
            chamberStates[i] = sensors.get(i).isBallDetected();
        }
    }

    public void checkBallEntry() {
        if (!chamberStates[0] && sensors.get(0).isBallDetected()) {
            totalNumBalls++;
        }
    }

    public void checkBallExit() {
        if (chamberStates[3] && !sensors.get(3).isBallDetected()) {
            totalNumBalls--;
        }
    }

    public void setSpeed(double frontSpeed, double backSpeed, double updrawSpeed) {
        this.frontSpeed = frontSpeed;
        this.backSpeed = backSpeed;
        this.updrawSpeed = updrawSpeed;
        run();
    }

    public void run() {
        frontMotor.set(frontSpeed);
        backMotor.set(backSpeed);
        updraw.set(ControlMode.PercentOutput, updrawSpeed);
    }

    public void setUpdrawSpeed(double updrawSpeed) {
        this.updrawSpeed = updrawSpeed;
    }

    public void stopMotors() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    public void toggleUpdraw() {
        double c = updraw.getMotorOutputPercent();
        if (MathUtils.doublesEqual(c, 0.0, 0.1)) {
            updraw(false);
        } else {
            stopUpdraw();
        }
    }

    /**
     * Run the uptake, emptying the top chamber of the indexer
     * 
     * @param reversed true to run updraw backwards, false to run normally
     */
    public void updraw(boolean reversed) {
        if (updrawStartTime == null) {
            updrawStartTime = System.currentTimeMillis();
        }
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
        double c = updraw.getMotorOutputPercent(); // We can't tell if it's actually up to speed, 
                                                   // but we're going off of "is it running"
        if (updrawStartTime == null) {
            return MathUtils.doublesEqual(c, UPDRAW_SPEED, 0.05);
        }

        return System.currentTimeMillis() >= updrawStartTime + 500;
    }

    @Override
    public HealthState checkHealth() { 
        //if either the rotator or updraw breaks, we can't use the indexer anymore.
        //Same deal if the indexer is jammed (but that's recoverable).
        CANError frontError = frontMotor.getLastError();
        CANError backError = backMotor.getLastError();

        boolean isFrontError = isSparkMaxErrored((SparkMAXLite) frontMotor);
        boolean isBackError = isSparkMaxErrored((SparkMAXLite) backMotor);
        // boolean isRotatorError = isSparkMaxErrored(rotator);
        boolean isUpdrawError = isPhoenixControllerErrored(updraw);
        if (isUpdrawError || isFrontError || isBackError) {
            MustangNotifications.reportError("RED Errors: front: %s, back: %s", frontError, backError);
            return HealthState.RED; 
        }
        // if the ToF sensor, (BallSensor) breaks but nothing else,
        // the next option would be manual control -- not a fatal issue
        for (int i = 0; i < sensors.size(); i++) {
            if (sensors.get(i) == null || !sensors.get(i).isHealthy()) {
                return HealthState.YELLOW;
            }
        }
        return HealthState.GREEN;
    }

    public void moveByPercentOutput(double output) {
        frontMotor.set(output);
        backMotor.set(output);
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
        setSpeed((SmartDashboard.getNumber("Front Motor Speed", 0.0)),
                (SmartDashboard.getNumber("Back Motor Speed", 0.0)), SmartDashboard.getNumber("Updraw Speed", 0.0));

        // updrawingMode = isUpdrawing();
        // if (updrawingMode && !isUpdrawing()) { // We were updrawing but no current spike is detected anymore
        //     updrawingMode = false;
        //     chamberStates[3] = false;
        // }
        // checkBallEntry();
        // checkBallExit();
        // updateChamberStates();
        // pushGameDataToDashboard();
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

    public boolean isFrontRunning() {
        return frontMotor.get() != 0;
    }

    public boolean isBackRunning() {
        return backMotor.get() != 0;
    }

    public int getTopChamber() {
        for (int i = 3; i >= 0; i--) {
            if (chamberStates[i])
                return i;
        }

        return -1; // no balls in
    }

    public int getBottomChamber() {
        for (int i = 0; i <= 3; i++) {
            if (chamberStates[i])
                return i;
        }

        return -1; // no balls in
    }

    public int getTotalNumBalls() {
        return totalNumBalls;
    }

    public double getFrontMotorSpeed() {
        return frontEncoder.getVelocity();
    }

    public double getBackMotorSpeed() {
        return backEncoder.getVelocity();
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
        // TODO: write smth for this
        return false;
    }
}