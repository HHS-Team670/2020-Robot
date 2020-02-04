package frc.team670.robot.utils.motorcontroller;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
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

    public static SparkMAXLite buildFactorySparkMAX(int deviceID) {
        return buildSparkMAX(deviceID, defaultConfig);
    }

    public static SparkMAXLite setPermanentFollower(int deviceID, CANSparkMax leader) {
        SparkMAXLite sparkMax = buildSparkMAX(deviceID, defaultFollowerConfig);
        sparkMax.follow(leader);
        return sparkMax;
    }

    public static SparkMAXLite buildSparkMAX(int deviceID, Config config) {
        SparkMAXLite sparkMax = new SparkMAXLite(deviceID);
        sparkMax.set(ControlType.kDutyCycle, 0.0);
        sparkMax.setInverted(config.INVERTED);
        return sparkMax;
    }

    /**
     * Used to build a pair of spark max controllers to control motors. Creates a leader on the port which is working and makes
     * other controller follow it
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @return motorPair a pair of motors with the first one as its leader and second one as the follower
     */
    public static List<SparkMAXLite> buildFactorySparkMAXPair(int motor1DeviceID, int motor2DeviceID) {
        return buildSparkMAXPair(motor1DeviceID, motor2DeviceID, defaultConfig, defaultConfig);
    }

     /**
     * Used to build a pair of spark max controllers to control motors. Creates a leader on the port which is working and makes
     * other controller follow it
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @param config The config to be set on to the motor controllers
     * @return motorPair a pair of motors with the first one as its leader and second one as the follower
     */
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, Config config) {
        return buildSparkMAXPair(motor1DeviceID, motor2DeviceID, config, config);
    }

    /**
     * Used to build a pair of spark max controllers to control motors. Creates a leader on the port which is working and makes
     * other controller follow it
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @param leaderConfig The config to be set on to the motor controller which is the leader
     * @param followerConfig The config to be set on to the motor controller which is the follower
     * @return motorPair a pair of motors with the first one as its leader and second one as the follower
     */
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, Config leaderConfig, Config followerConfig) {
        SparkMAXLite sparkMaxLeader = buildSparkMAX(motor1DeviceID, leaderConfig);
        SparkMAXLite sparkMaxFollower;
        
        if (sparkMaxLeader.getLastError() != CANError.kOk && sparkMaxLeader.getLastError() != null) {
            sparkMaxLeader = buildSparkMAX(motor2DeviceID, leaderConfig);
            sparkMaxFollower = buildSparkMAX(motor1DeviceID, followerConfig);
            sparkMaxFollower.follow(sparkMaxLeader);
            List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
            Logger.consoleLog("Primary Spark Max Broken. Switching to SparkMax id %s", sparkMaxLeader.getDeviceId());
            return motorPair;
        }
        else{
            sparkMaxFollower = buildSparkMAX(motor2DeviceID, followerConfig);
            sparkMaxFollower.follow(sparkMaxLeader);
            List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
            Logger.consoleLog("Primary Spark Max Working. SparkMax Leader id is %s", sparkMaxLeader.getDeviceId());
            return motorPair;
        }
    }

}
