package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.MotorConfig;
import frc.team670.robot.utils.motorcontroller.TalonSRXLite;

/**
 * Represents the ball indexer subsystem, which tracks and stores up to 5 balls.
 * 
 * @author ctychen, eddieli, ruchidixit
 */
public class Indexer extends SparkMaxRotatingSubsystem {

    private TalonSRXLite updraw;

    private CANEncoder encoder;
    private boolean[] chamberStates;
    private double current;
    private double prevCurrent;
    private double currentChange;

    private boolean ballIsUpdrawing;
    private boolean ballHasLeft;

    // Current control: updraw
    // TODO: find all these values
    private static final double UPDRAW_CURR_P = 0.0;
    private static final double UPDRAW_CURR_I = 0;
    private static final double UPDRAW_CURR_D = 0;
    private static final double UPDRAW_CURR_FF = 0.2;

    private static final double UPDRAW_CURRENT = 0;

    private static final int UPDRAW_CURRENT_SLOT = 0;

    private static final double UPDRAW_V_P = 0;
    private static final double UPDRAW_V_I = 0;
    private static final double UPDRAW_V_D = 0;
    private static final double UPDRAW_V_FF = 0.2;

    private static final int UPDRAW_VELOCITY_SLOT = 1;

    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 0;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 0;

    private static final int INDEXER_TICKS_PER_ROTATION = 42; // NEO550 integrated encoder is 42 counts per rev

    private static final double INDEXER_DEGREES_PER_CHAMBER = 72;

    private static final int CHAMBER_0_AT_TOP_POS_IN_DEGREES = 252;
    private static final int CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES = 72;


    /**
     * PID and SmartMotion constants for the indexer rotator go here.
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.INDEXER_ROTATOR;
        }

        public int getSlot() {
            return 0;
        }

        public MotorConfig.Motor_Type getMotorType() {
            return MotorConfig.Motor_Type.NEO_550;
        }

        public double getP() {
            return 0.001;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
        }

        public double getFF() {
            return 0;
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
            return 2000;
        }

        public double getMinVelocity() {
            return 0;
        }

        public double getMaxAcceleration() {
            return 1500;
        }

        public double getAllowedError() {
            return 50;
        }

        public float getForwardSoftLimit() {
            return 1000;
        }

        public float getReverseSoftLimit() {
            return -1000;
        }

        public int getContinuousCurrent() {
            return 3;
        }

        public int getPeakCurrent() {
            return 6;
        }

        @Override
        public double getRotatorGearRatio() {
            return 100; // TODO: Update when we find out for sure
        }

    }

    public static final Config INDEXER_CONFIG = new Config();

    private SensorCollection updrawSensors;

    public Indexer() {
        super(INDEXER_CONFIG);

        this.updraw = new TalonSRXLite(RobotMap.UPDRAW_SPINNER);

        chamberStates = new boolean[5];

        updraw.setNeutralMode(NeutralMode.Coast);
        this.updrawSensors = updraw.getSensorCollection();

        updraw.config_kP(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_P);
        updraw.config_kI(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_I);
        updraw.config_kD(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_D);
        updraw.config_kF(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_FF);

        updraw.config_kP(UPDRAW_VELOCITY_SLOT, UPDRAW_V_P);
        updraw.config_kI(UPDRAW_VELOCITY_SLOT, UPDRAW_V_I);
        updraw.config_kD(UPDRAW_VELOCITY_SLOT, UPDRAW_V_D);
        updraw.config_kF(UPDRAW_VELOCITY_SLOT, UPDRAW_V_FF);

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);
    }

    public int totalNumOfBalls() {
        int totalNumBalls = 0;
        for (boolean c : chamberStates) {
            if (c) {
                totalNumBalls++;
            }
        }
        return totalNumBalls;
    }

    // TODO: figure this out once we have sensor(s)
    public void setChamberStates() {
        chamberStates[getBottomChamber()] = true;
    }

    public void prepareToIntake() {
        setTargetAngleInDegrees(INDEXER_DEGREES_PER_CHAMBER * getIntakeChamber() + CHAMBER_0_AT_TOP_POS_IN_DEGREES);
    }

    public void prepareToShoot() {
        setTargetAngleInDegrees(getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES);
    }

    /**
     * uptake into shooter
     * 
     * @param percentOutput percent output for the updraw
     * @post top chamber should be empty
     */
    public void uptake(double percentOutput) {
        updraw.set(ControlMode.PercentOutput, percentOutput);
        //chamberStates[getTopChamber()] = false;
    }

    // TODO: does this work
    public boolean updrawIsUpToSpeed() {
        double c = updraw.getMotorOutputPercent();
        return (Math.abs(c-0.5) < 0.0005);
    }

    // TODO: check that ball has left chamber using current???

    public void rotateToLoadShoot() {
        setTargetAngleInDegrees((getShootChamber() * INDEXER_DEGREES_PER_CHAMBER + CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES)
        - INDEXER_DEGREES_PER_CHAMBER/2);
    }

    private int getTopChamber() {

        double pos = getPosition() % 1.0;
        if (pos < 0) {
            pos++;
        }
        if (pos <= 0.2 && pos >= 0) {
            return 2;
        } else if (pos <= 0.4 && pos >= 0.2) {
            return 1;
        } else if (pos <= 0.6 && pos >= 0.4) {
            return 0;
        } else if (pos <= 0.8 && pos > 0.6) {
            return 4;
        } else if (pos >= 0.8) {
            return 3;
        } else {
            return -1;
        }
    }

    // chamber currently at the top
    // if its exactly between two chambers, default to later before
    private int getBottomChamber() {

        double pos = getPosition() % 1.0;
        if (pos < 0) {
            pos++;
        }
        if (pos <= 0.1 || pos >= 0.9) {
            return 0;
        } else if (pos <= 0.3 && pos >= 0.1) {
            return 4;
        } else if (pos <= 0.5 && pos >= 0.3) {
            return 3;
        } else if (pos <= 0.7 && pos > 0.5) {
            return 2;
        } else if (pos <= 0.9 && pos >= 0.7) {
            return 1;
        } else {
            return -1;
        }
    }

    // Designed by JOSHIE SANGYALSWEIO
    private int getIntakeChamber() {
        if (totalNumOfBalls() == 5) {
            return -1;
        }
        int currentBottom = getBottomChamber();
        boolean foundFilled = false;
        for (int i = currentBottom; i < currentBottom + 5; i++) {
            if (foundFilled && !chamberStates[i % 5]) {
                return i % 5;
            }
            if (chamberStates[i % 5]) {
                foundFilled = true;
            }

        }
        return currentBottom;
    }

    private int getShootChamber() {
        if (totalNumOfBalls() == 5) {
            return getTopChamber();
        }
        int currentTop = getTopChamber();
        boolean foundFilled = false;
        for (int i = currentTop; i < currentTop + 5; i++) {
            if (foundFilled && !chamberStates[i % 5]) {
                return (i - 1) % 5;
            }
            if (chamberStates[i % 5]) {
                foundFilled = true;
            }

        }
        return currentTop;
    }

    public void zeroRevolver() {
        setTargetAngleInDegrees(CHAMBER_0_AT_BOTTOM_POS_IN_DEGREES);
    }

    public boolean ballIn() {
        // FSR maybe?
        return false;
    }

    /**
     * Sets speed of the motor controlling the revolver
     */
    public void setSpeed(double speed) {
        rotator.set(speed);
    }

    public double getSpeed() {
        return rotator.get();
    }

    public double getPosition() {
        return encoder.getPosition() / ROTATOR_GEAR_RATIO;
    }

    @Override
    public HealthState checkHealth() {
        if (isSparkMaxErrored(rotator)){
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void moveByPercentOutput(double output) {

    }

    @Override
    public double getCurrentAngleInDegrees() {
        return (getUnadjustedPosition() / INDEXER_TICKS_PER_ROTATION);
    }

    @Override
    public void mustangPeriodic() {
        prevCurrent = current;
        current = updraw.getSupplyCurrent();
        currentChange = current - prevCurrent;

        if (currentChange > 0){
            
        }

    }

}