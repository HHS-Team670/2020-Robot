package frc.team670.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.commands.joystickControls.JoystickTurret;
import frc.team670.robot.constants.RobotMap;


public class Turret extends SparkMaxRotatingSubsystem {

    // TODO: Set these values. Keeping it small right now for testing.

    // Turret pointing straight forward is 180 degrees
    private static final double TURRET_MIN_DEGREES = -240; // all the way back
    private static final double TURRET_MAX_DEGREES = 18; // from front, past straight forward

    private static final double SOFT_MINIMUM_DEGREES = TURRET_MIN_DEGREES + 3;
    private static final double SOFT_MAXIMUM_DEGREES = 22; // can go past max 0ing point/sensor, that's only for zeroing. This is needed for left auton path

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

        public double getMaxRotatorRPM() {
            return 170; // TODO: probably needs to be adjusted
        }

        public double getMinRotatorRPM() {
            return 0;
        }

        public double getMaxAcceleration() {
            return 170; // TODO: probably needs to be adjusted
        }

        public double getAllowedError() {
            // equivalent of 0.25 degrees, in rotations
            return (0.5 / 360) * this.getRotatorGearRatio();
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

        public double getMaxVelocity() {
            // TODO Auto-generated method stub
            return 0.0;
        }

        public double getMinVelocity() {
            // TODO Auto-generated method stub
            return 0.0;
        }

    }

    public static final Config turretConfig = new Config();
    private final double DEGREES_PER_MOTOR_ROTATION = 360 / turretConfig.getRotatorGearRatio();

    private Vision vision;
    private LEDSubsystem leds;

    public Turret(Vision vision, LEDSubsystem leds) {
        super(turretConfig);
        rotator.setInverted(true);
        this.vision = vision;
        this.leds = leds;
        forwardLimit = rotator.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        reverseLimit = rotator.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    }

    @Override
    public HealthState checkHealth() {
        if (isSparkMaxErrored(rotator)) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    public boolean hasZeroed() {
        return this.zeroedAlready;
    }

    public void setLimitSwitch(boolean enabled){
        forwardLimit.enableLimitSwitch(enabled);
        reverseLimit.enableLimitSwitch(enabled);
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
        // Logger.consoleLog("Forward: %s Backward %s", isForwardLimitSwitchxTripped(), isReverseLimitSwitchTripped());
        // Logger.consoleLog("Turret perioidic");
        SmartDashboard.putNumber("Turret Angle", getCurrentAngleInDegrees());
        if(hasZeroed() && vision.hasTarget()){
            setSystemTargetAngleInDegrees(relativeAngleToAbsoluteInDegrees(vision.getAngleToTarget()));
            // Logger.consoleLog("Auto align");
        }
        if(vision.hasTarget()){
            if(super.hasReachedTargetPosition()){
                leds.setGreenBuffer();
            }
            else{
                leds.setBlueBuffer();
            }
        }
        else{
            leds.setRedBuffer();
        }



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
            rotator_encoder.setPosition(getMotorRotationsFromAngle(TURRET_MAX_DEGREES));
            this.rotator.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)(getMotorRotationsFromAngle(SOFT_MAXIMUM_DEGREES)));
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