package frc.team670.robot.utils.motorcontroller;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.ParamEnum;

/**
 * Utility class for configuring a SparkMAX to default settings and resetting to
 * factory defaults.
 * 
 * @author ruchidixit
 */
public class VictorSPXFactory {

    public static final int TIMEOUT_MS = 100;

    public static class Config {

        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public double NEUTRAL_DEADBAND = 0.04; // factory default

        public boolean INVERTED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = true;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

    }

    public static final Config defaultConfig = new Config();
    public static final Config defaultFollowerConfig = new Config();

    static {
        defaultFollowerConfig.CONTROL_FRAME_PERIOD_MS = 100;
        defaultFollowerConfig.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        defaultFollowerConfig.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    public static VictorSPXLite buildFactoryVictorSPX(int deviceID, boolean invert) {
        return buildVictorSPX(deviceID, defaultConfig, invert);
    }

    public static VictorSPXLite buildVictorSPX(int deviceID, Config config, boolean invert) {
        VictorSPXLite VictorSPXLite = new VictorSPXLite(deviceID);
        VictorSPXLite.set(ControlMode.PercentOutput, 0.0);
        VictorSPXLite.setInverted(invert);

        VictorSPXLite.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        VictorSPXLite.clearMotionProfileHasUnderrun(TIMEOUT_MS);
        VictorSPXLite.clearMotionProfileTrajectories();

        VictorSPXLite.clearStickyFaults(TIMEOUT_MS);

        // Turn off re-zeroing by default.
        VictorSPXLite.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, TIMEOUT_MS);
        VictorSPXLite.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TIMEOUT_MS);

        VictorSPXLite.configNominalOutputForward(0, TIMEOUT_MS);
        VictorSPXLite.configNominalOutputReverse(0, TIMEOUT_MS);
        VictorSPXLite.configNeutralDeadband(config.NEUTRAL_DEADBAND, TIMEOUT_MS);

        VictorSPXLite.configPeakOutputForward(1.0, TIMEOUT_MS);
        VictorSPXLite.configPeakOutputReverse(-1.0, TIMEOUT_MS);

        VictorSPXLite.setNeutralMode(config.NEUTRAL_MODE);

        VictorSPXLite.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, TIMEOUT_MS);
        VictorSPXLite.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);

        VictorSPXLite.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, TIMEOUT_MS);
        VictorSPXLite.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);
        VictorSPXLite.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        VictorSPXLite.selectProfileSlot(0, 0);

        VictorSPXLite.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TIMEOUT_MS);
        VictorSPXLite.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, TIMEOUT_MS);

        VictorSPXLite.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, TIMEOUT_MS);
        VictorSPXLite.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, TIMEOUT_MS);
        VictorSPXLite.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return VictorSPXLite;
    }

}