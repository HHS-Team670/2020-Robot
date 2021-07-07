package frc.team670.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;

import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.robot.commands.joystickControls.JoystickTurret;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;


public class Turret extends SparkMaxRotatingSubsystem {

    // TODO: Set these values. Keeping it small right now for testing.

    // Turret pointing straight forward is 180 degrees
    private static final double TURRET_MIN_DEGREES = -240; // all the way back
    private static final double TURRET_MAX_DEGREES = 32; //12 // from front, past straight forward

    private static final double LIMIT_SWITCH_DEGREES = 12;

    private static final double SOFT_MINIMUM_DEGREES = TURRET_MIN_DEGREES + 3;
    private static final double SOFT_MAXIMUM_DEGREES = 32;

    private CANDigitalInput forwardLimit;
    private CANDigitalInput reverseLimit;

    private boolean zeroedAlready = false;

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
            // equivalent of 0.25 degrees, in rotations
            return (0.25 / 360) * this.getRotatorGearRatio();
        }

        public boolean enableSoftLimits() {
            return false;
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
        // return HealthState.RED;
    }

    public boolean hasZeroed() {
        return this.zeroedAlready;
    }

    /**
     * @param zeroedAlready the zeroedAlready to set
     */
    public void setZeroedAlready(boolean zeroedAlready) {
        this.zeroedAlready = zeroedAlready;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void setSystemTargetAngleInDegrees(double targetAngle) {
        if (targetAngle > SOFT_MAXIMUM_DEGREES || targetAngle < SOFT_MINIMUM_DEGREES) {
            Logger.consoleLog("Turret angle is out of range, angle is %s", targetAngle);
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
            rotator_encoder.setPosition(getMotorRotationsFromAngle(LIMIT_SWITCH_DEGREES));
            this.rotator.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)(getMotorRotationsFromAngle(TURRET_MAX_DEGREES)));
        }

        if (isReverseLimitSwitchTripped()) {
            rotator_encoder.setPosition(getMotorRotationsFromAngle(TURRET_MIN_DEGREES));
            this.rotator.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)(getMotorRotationsFromAngle(TURRET_MIN_DEGREES)));
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
        Logger.consoleLog("Turret defaulted to joystick");
    }

    @Override
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

}