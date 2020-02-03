package frc.team670.robot.utils.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.ParamEnum;

/**
 * Utility class for configuring a TalonSRX to default settings and resetting to
 * factory defaults.
 * 
 * @author ctychen, ruchidixit
 */
public class TalonSRXFactory {

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

        // TODO: figure out some values for this config, because I dropped something
        // random...might want to change these?
        static {
                defaultFollowerConfig.CONTROL_FRAME_PERIOD_MS = 100;
                defaultFollowerConfig.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
                defaultFollowerConfig.GENERAL_STATUS_FRAME_RATE_MS = 1000;
                defaultFollowerConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
                defaultFollowerConfig.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
                defaultFollowerConfig.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
                defaultFollowerConfig.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        }

        /**
         * Create a TalonSRX with factory settings, as well as configuring other
         * settings to a usable default.
         */
        public static TalonSRX buildFactoryTalonSRX(int deviceID) {
                return buildTalonSRX(deviceID, defaultConfig);
        }

        /**
         * 
         * @param deviceID ID of the follower TalonSRX
         * @param masterID ID of the TalonSRX for this controller to follow
         * @return TalonSRX set to follow the 'master', with appropriate follower
         *         settings as well
         */
        public static TalonSRX setPermanentFollower(int deviceID, int masterID) {
                TalonSRX talonsrx = buildTalonSRX(deviceID, defaultFollowerConfig);
                talonsrx.set(ControlMode.Follower, masterID);
                return talonsrx;
        }

        public static TalonSRX buildTalonSRX(int deviceID, Config config) {
                TalonSRX talonsrx = new TalonSRXLite(deviceID);
                talonsrx.set(ControlMode.PercentOutput, 0.0);
                talonsrx.setInverted(config.INVERTED);

                talonsrx.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
                talonsrx.clearMotionProfileHasUnderrun(TIMEOUT_MS);
                talonsrx.clearMotionProfileTrajectories();

                talonsrx.clearStickyFaults(TIMEOUT_MS);

                talonsrx.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled,
                                TIMEOUT_MS);
                talonsrx.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled,
                                TIMEOUT_MS);

                // Turn off re-zeroing by default.
                talonsrx.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, TIMEOUT_MS);
                talonsrx.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TIMEOUT_MS);

                talonsrx.configNominalOutputForward(0, TIMEOUT_MS);
                talonsrx.configNominalOutputReverse(0, TIMEOUT_MS);
                talonsrx.configNeutralDeadband(config.NEUTRAL_DEADBAND, TIMEOUT_MS);

                talonsrx.configPeakOutputForward(1.0, TIMEOUT_MS);
                talonsrx.configPeakOutputReverse(-1.0, TIMEOUT_MS);

                talonsrx.setNeutralMode(config.NEUTRAL_MODE);

                talonsrx.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, TIMEOUT_MS);
                talonsrx.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);

                talonsrx.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, TIMEOUT_MS);
                talonsrx.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);
                talonsrx.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

                talonsrx.selectProfileSlot(0, 0);

                talonsrx.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TIMEOUT_MS);
                talonsrx.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
                                TIMEOUT_MS);

                talonsrx.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, TIMEOUT_MS);
                talonsrx.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, TIMEOUT_MS);

                talonsrx.configVoltageCompSaturation(0.0, TIMEOUT_MS);
                talonsrx.configVoltageMeasurementFilter(32, TIMEOUT_MS);
                talonsrx.enableVoltageCompensation(false);

                talonsrx.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);

                talonsrx.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS,
                                TIMEOUT_MS);
                talonsrx.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                                config.FEEDBACK_STATUS_FRAME_RATE_MS, TIMEOUT_MS);

                talonsrx.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
                                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
                talonsrx.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
                talonsrx.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, TIMEOUT_MS);

                talonsrx.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

                return talonsrx;
        }

}