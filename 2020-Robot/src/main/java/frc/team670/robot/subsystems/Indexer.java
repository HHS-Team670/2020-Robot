package frc.team670.robot.subsystems;

import java.sql.Time;
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
import frc.team670.mustanglib.dataCollection.sensors.Multiplexer;
import frc.team670.mustanglib.dataCollection.sensors.TimeOfFlightSensor;
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

    private boolean[] chamberStates;

    private int frontExceededCurrentLimitCount = 0;
    private int backExceededCurrentLimitCount = 0;

    private Long updrawStartTime;

    private boolean updrawingMode;

    private double UPDRAW_SPEED = -0.9;
    private double INDEXER_SPEED = 0.9;

    private int isUpdrawingCount = 0;
    private int totalNumBalls;

    // Updraw normally at around/below 9A when it's just spinning, peaks when ball
    // touches
    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 9;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 15;
    
    private TimeOfFlightSensor chamber0Sensor, chamber1Sensor, chamber2Sensor;

    private Multiplexer multiplexer;

    public Indexer(Multiplexer multiplexer, Conveyor conveyor) {
        super();

        frontMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.FRONT_MOTOR, Motor_Type.NEO_550);
        backMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.BACK_MOTOR, Motor_Type.NEO_550);
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();
        this.conveyor = conveyor;

        // Updraw should be inverted
        this.updraw = TalonSRXFactory.buildFactoryTalonSRX(RobotMap.UPDRAW_SPINNER, false);

        this.multiplexer = multiplexer;
        
        chamber0Sensor = new TimeOfFlightSensor(RobotMap.INDEXER_MUL_PORT, 0, 20);
        chamber1Sensor = new TimeOfFlightSensor(RobotMap.INDEXER_MUL_PORT, 1, 20);
        chamber2Sensor = new TimeOfFlightSensor(RobotMap.INDEXER_MUL_PORT, 2, 20);

        multiplexer.attachSensor(chamber0Sensor, chamber1Sensor, chamber2Sensor);

        chamberStates = new boolean[4];

        updraw.setNeutralMode(NeutralMode.Coast); // free, nothing holding instead of brake

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);

        updraw.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        updraw.enableVoltageCompensation(true);
    }

    private void pushGameDataToDashboard() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("/SmartDashboard");
        NetworkTableEntry gameData = table.getEntry("Balls");
        gameData.forceSetNumber(totalNumBalls);
    }

    /**
     * Updates the states of the chambers
     */
    public void updateChamberStates() {
        for (int i = 0; i < multiplexer.getSensors().size(); i++) {
            chamberStates[i] = multiplexer.getSensors().get(i).isObjectWithinThreshold();
        }
    }

    public void checkBallEntry() {
        if (!chamberStates[0] && multiplexer.getSensors().get(0).isObjectWithinThreshold()) {
            totalNumBalls++;
        }
    }

    public void checkBallExit() {
        if (chamberStates[3] && !multiplexer.getSensors().get(3).isObjectWithinThreshold()) {
            totalNumBalls--;
        }
    }

    public void run() {
        frontMotor.set(INDEXER_SPEED);
        backMotor.set(-1 * INDEXER_SPEED);
        updraw.set(ControlMode.PercentOutput, UPDRAW_SPEED);
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
        for (int i = 0; i < multiplexer.getSensors().size(); i++) {
            if (multiplexer.getSensors().get(i) == null || !multiplexer.getSensors().get(i).isHealthy()) {
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
    //     int i = 0;
    // if(i==5){
    //   Logger.consoleLog("Sensor0: %s Sensor1: %s Sensor2: %s", multiplexer.getSensors().get(0).getDistance(), multiplexer.getSensors().get(1).getDistance(), multiplexer.getSensors().get(2).getDistance());
    //   i=0;
    // }
    // i++;
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
        return multiplexer.getSensors().get(chamber).getDistance() < RobotConstants.INDEXER_WIDTH;
    }

    public boolean isFrontRunning() {
        return frontMotor.get() != 0;
    }

    public boolean isBackRunning() {
        return backMotor.get() != 0;
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

    public TimeOfFlightSensor getSensor(int sensor) {
        return multiplexer.getSensors().get(sensor);
    }
}