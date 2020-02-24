package frc.team670.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.team670.robot.commands.MustangScheduler;
import frc.team670.robot.commands.joystickControls.JoystickTurret;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

public class Turret extends SparkMaxRotatingSubsystem {

    // TODO: Set these values. Keeping it small right now for testing.
    public static final int SOFT_MINIMUM_DEGREES = -50;
    public static final int SOFT_MAXIMUM_DEGREES = 50;

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
        rotator_encoder.setPosition(0);
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
     * @param speed to set turret to, [1, 1]
     */
    @Override
    public void moveByPercentOutput(double output) {
        this.rotator.set(output);
    }

    public void initDefaultCommand(){
        MustangScheduler.getInstance().setDefaultCommand(this, new JoystickTurret(this));
    }

}