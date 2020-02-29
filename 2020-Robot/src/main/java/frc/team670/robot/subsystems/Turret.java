package frc.team670.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANError;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.team670.robot.commands.MustangScheduler;
import frc.team670.robot.commands.joystickControls.JoystickTurret;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

public class Turret extends SparkMaxRotatingSubsystem {

    // TODO: Set these values. Keeping it small right now for testing.

    private static final int TURRET_MIN_DEGREES = -120;
    private static final int TURRET_MAX_DEGREES = 120;

    private static final int SOFT_MINIMUM_DEGREES = TURRET_MIN_DEGREES + 5;
    private static final int SOFT_MAXIMUM_DEGREES = TURRET_MAX_DEGREES - 5;

    private CANDigitalInput forwardLimit;
    private CANDigitalInput reverseLimit;

    public String kEnable;
    public String kDisable;

    /**
     * Constants for the turret, including PIDF and SmartMotion values.
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.TURRET_ROTATOR;
        }

        public IdleMode setRotatorIdleMode() {
            return IdleMode.kBrake;
        }

        // Slot for SmartMotion control
        public int getSlot() {
            return 0;
        }

        public double getP() {
            return 0.00005;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
        }

        public double getFF() {
            return 0.0012;
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

        public double getMaxVelocity() {
            return 200; // TODO: probably needs to be adjusted
        }

        public double getMinVelocity() {
            return 0;
        }

        public double getMaxAcceleration() {
            return 200; // TODO: probably needs to be adjusted
        }

        public double getAllowedError() {
            // equivalent of 2 degrees, in rotations
            return (2 / 360) * this.getRotatorGearRatio();
        }

        public boolean enableSoftLimits() {
            return true;
        }

        public float[] setSoftLimits() {
            return new float[] { (float) (SOFT_MINIMUM_DEGREES * getRotatorGearRatio()),
                    (float) (SOFT_MAXIMUM_DEGREES * getRotatorGearRatio()) };
        }

        public int getContinuousCurrent() {
            return 3; // Placeholder
        }

        public int getPeakCurrent() {
            return 6; // Placeholder
        }

        public Motor_Type getMotorType() {
            return Motor_Type.NEO_550;
        }

        public double getRotatorGearRatio() {
            return 76.611;
        }

    }

    public static final Config turretConfig = new Config();
    private final double DEGREES_PER_MOTOR_ROTATION = 360 / turretConfig.getRotatorGearRatio();

    public Turret() {
        super(turretConfig);
        rotator.setInverted(true);
        forwardLimit = rotator.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        reverseLimit = rotator.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit.enableLimitSwitch(true);
    }

    @Override
    public HealthState checkHealth() {
        if (isSparkMaxErrored(rotator)) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void setSystemTargetAngleInDegrees(double targetAngle) {
        if (targetAngle > SOFT_MAXIMUM_DEGREES || targetAngle < SOFT_MINIMUM_DEGREES) {
            throw new IllegalArgumentException(
                    "Invalid angle: must be within range " + SOFT_MINIMUM_DEGREES + " and " + SOFT_MAXIMUM_DEGREES);
        } else {
            super.setSystemTargetAngleInDegrees(targetAngle);
        }
    }

    @Override
    public double getCurrentAngleInDegrees() {
        return getUnadjustedPosition() * DEGREES_PER_MOTOR_ROTATION;
    }

    public double relativeAngleToAbsoluteInDegrees(double relAngle) {
        return getCurrentAngleInDegrees() + relAngle;
    }

    /**
     * 
     * @return true if the reverse limit was hit, and false if not. Note that
     *         magnetic limit switches are active-low sensors, so when they are
     *         tripped we get 0V on signal, but specifying this polarity when
     *         creating them should deal with that
     */
    public boolean isReverseLimitSwitchTripped() {
        return reverseLimit.get();
    }

    /**
     * 
     * @return true if the forward limit was hit, and false if not. Note that
     *         magnetic limit switches are active-low sensors, so when they are
     *         tripped we get 0V on signal, but specifying this polarity when
     *         creating them should deal with that
     */
    public boolean isForwardLimitSwitchTripped() {
        return forwardLimit.get();
    }

    /**
     * Sets zero'd position for built in encoder to one of the limits based on which
     * one we move to for zeroing
     */
    public void resetRotatorEncoderFromLimitSwitch() {
        if (isForwardLimitSwitchTripped()) {
            rotator_encoder.setPosition(getMotorRotationsFromAngle(TURRET_MAX_DEGREES));
        }

        if (isReverseLimitSwitchTripped()) {
            rotator_encoder.setPosition(getMotorRotationsFromAngle(TURRET_MIN_DEGREES));
        }
    }

    /**
     * @param speed to set turret to, [1, 1]
     */
    @Override
    public void moveByPercentOutput(double output) {
        this.rotator.set(output);
    }

    public void initDefaultCommand() {
        MustangScheduler.getInstance().setDefaultCommand(this, new JoystickTurret(this));
    }

}