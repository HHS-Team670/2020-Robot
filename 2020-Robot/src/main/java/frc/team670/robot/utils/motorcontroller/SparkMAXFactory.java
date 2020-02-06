package frc.team670.robot.utils.motorcontroller;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.team670.robot.utils.Logger;

import com.revrobotics.ControlType;

/**
 * Utility class for configuring a SparkMAX to default settings and resetting to
 * factory defaults.
 * 
 * @author ctychen, ruchidixit
 */
public class SparkMAXFactory {

    public static class Config {

        public boolean BURN_FACTORY_DEFAULT_FLASH = false;
        public IdleMode DEFAULT_MODE = IdleMode.kCoast;
        public boolean INVERTED = false;

        public int STATUS_FRAME_0_RATE_MS = 10;
        public int STATUS_FRAME_1_RATE_MS = 1000;
        public int STATUS_FRAME_2_RATE_MS = 1000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public boolean ENABLE_VOLTAGE_COMPENSATION = false;
        public double NOMINAL_VOLTAGE = 12.0;

    }

    public static final Config defaultConfig = new Config();
    public static final Config defaultFollowerConfig = new Config();

    static {
        defaultFollowerConfig.STATUS_FRAME_0_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_1_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_2_RATE_MS = 1000;
    }

    /**
     * Creates a SparkMAXLite with factory settings.
     */
    public static SparkMAXLite buildFactorySparkMAX(int deviceID, MotorConfig.Motor_Type motorType) {
        return buildSparkMAX(deviceID, defaultConfig, motorType);
    }

    public static SparkMAXLite setPermanentFollower(int deviceID, SparkMAXLite leader) {
        SparkMAXLite sparkMax = buildSparkMAX(deviceID, defaultFollowerConfig, leader.getMotor());
        sparkMax.follow(leader);
        return sparkMax;
    }

    /**
     * 
     * @param deviceID  CAN ID of this SparkMax
     * @param config    The configuration to set this for, ex. default or
     *                  defaultFollower
     * @param motorType The kind of motor this controller will be using
     * @return SparkMAXLite set to this configuration with current limit
     */
    public static SparkMAXLite buildSparkMAX(int deviceID, Config config, MotorConfig.Motor_Type motorType) {
        SparkMAXLite sparkMax = new SparkMAXLite(deviceID, motorType);
        sparkMax.set(ControlType.kDutyCycle, 0.0);
        sparkMax.setInverted(config.INVERTED);
        sparkMax.setSmartCurrentLimit(MotorConfig.MOTOR_MAX_CURRENT.get(motorType));
        return sparkMax;
    }

    /**
     * Used to build a pair of spark max controllers to control motors. Creates a
     * leader on the port which is working and makes other controller follow it
     * 
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @return motorPair a pair of motors with the first one as its leader and
     *         second one as the follower
     */
    public static List<SparkMAXLite> buildFactorySparkMAXPair(int motor1DeviceID, int motor2DeviceID, MotorConfig.Motor_Type motorType) {
        return buildSparkMAXPair(motor1DeviceID, motor2DeviceID, defaultConfig, defaultConfig, motorType);
    }

    /**
     * Used to build a pair of spark max controllers to control motors. Creates a
     * leader on the port which is working and makes other controller follow it
     * 
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @param config         The config to be set on to the motor controllers
     * @return motorPair a pair of motors with the first one as its leader and
     *         second one as the follower
     */
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, Config config, MotorConfig.Motor_Type motorType){
        return buildSparkMAXPair(motor1DeviceID, motor2DeviceID, config, config, motorType);
    }

    /**
     * Used to build a pair of spark max controllers to control motors. Creates a
     * leader on the port which is working and makes other controller follow it
     * 
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @param leaderConfig   The config to be set on to the motor controller which
     *                       is the leader
     * @param followerConfig The config to be set on to the motor controller which
     *                       is the follower
     * @return motorPair a pair of motors with the first one as its leader and
     *         second one as the follower
     */
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, Config leaderConfig,
            Config followerConfig, MotorConfig.Motor_Type motorType) {
        SparkMAXLite sparkMaxLeader = buildSparkMAX(motor1DeviceID, leaderConfig, motorType);
        SparkMAXLite sparkMaxFollower;

        if (sparkMaxLeader.getLastError() != CANError.kOk && sparkMaxLeader.getLastError() != null) {
            sparkMaxLeader = buildSparkMAX(motor2DeviceID, leaderConfig, motorType);
            sparkMaxFollower = buildSparkMAX(motor1DeviceID, followerConfig, motorType);
            sparkMaxFollower.follow(sparkMaxLeader);
            List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
            Logger.consoleLog("Primary Spark Max Broken. Switching to SparkMax id %s", sparkMaxLeader.getDeviceId());
            return motorPair;
        } else {
            sparkMaxFollower = buildSparkMAX(motor2DeviceID, followerConfig, motorType);
            sparkMaxFollower.follow(sparkMaxLeader);
            List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
            Logger.consoleLog("Primary Spark Max Working. SparkMax Leader id is %s", sparkMaxLeader.getDeviceId());
            return motorPair;
        }
    }

}