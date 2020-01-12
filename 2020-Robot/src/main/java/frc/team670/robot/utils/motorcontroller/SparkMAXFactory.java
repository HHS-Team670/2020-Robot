package frc.team670.robot.utils.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

/**
 * Utility class for configuring a SparkMAX to default settings and resetting to factory defaults.
 * @author ctychen, ruchidixit 
 */
public class SparkMAXFactory{

    public static class Config{

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


    public static CANSparkMax buildFactorySparkMAX(int deviceID){
        return buildSparkMAX(deviceID, defaultConfig);
    }

    public static CANSparkMax setPermanentFollower(int deviceID, CANSparkMax master){
        CANSparkMax sparkMax = buildSparkMAX(deviceID, defaultFollowerConfig);
        sparkMax.follow(master);
        return sparkMax;
    }

    public static CANSparkMax buildSparkMAX(int deviceID, Config config) {
        SparkMAXLite sparkMax = new SparkMAXLite(deviceID);
        sparkMax.set(ControlType.kDutyCycle, 0.0);
        sparkMax.setInverted(config.INVERTED);
        return sparkMax;
    }

}
