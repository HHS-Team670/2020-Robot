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

    public static List<SparkMAXLite> buildSparkMAXPair(int deviceIDMotor1, int deviceIDMotor2) {
        SparkMAXLite sparkMaxLeader = new SparkMAXLite(deviceIDMotor1);
        SparkMAXLite sparkMaxFollower;
        
        if (sparkMaxLeader.getLastError() != CANError.kOk && sparkMaxLeader.getLastError() != null) {
            sparkMaxLeader = buildSparkMAX(deviceIDMotor2, defaultConfig);
            sparkMaxFollower = buildSparkMAX(deviceIDMotor1, defaultConfig);
            sparkMaxFollower.follow(sparkMaxLeader);
            List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
            Logger.consoleLog("Primary Spark Max Broken. Switching to SparkMax id %s", sparkMaxLeader.getDeviceId());
            return motorPair;
        }
        else{
            sparkMaxLeader = buildSparkMAX(deviceIDMotor1, defaultConfig);
            sparkMaxFollower = buildSparkMAX(deviceIDMotor2, defaultConfig);
            sparkMaxFollower.follow(sparkMaxLeader);
            List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
            Logger.consoleLog("Primary Spark Max Working. SparkMax Leader id is %s", sparkMaxLeader.getDeviceId());
            return motorPair;
        }
    }

}
