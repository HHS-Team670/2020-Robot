package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.TalonSRXLite;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 */
public class Indexer extends SparkMaxRotatingSubsystem {

    private CANSparkMax rotator;
    private TalonSRXLite updraw;

    private CANEncoder encoder;
    private boolean[] chamberStates;
    private CANPIDController controller;

    // Current control: updraw
    // TODO: find all these values
    private static final double UPDRAW_CURR_P = 0.2;
    private static final double UPDRAW_CURR_I = 0;
    private static final double UPDRAW_CURR_D = 0;

    private static final int UPDRAW_CURRENT_SLOT = 0;

    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 0;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 0;

    private static final int INDEXER_TICKS_PER_ROTATION = 42; // NEO550 integrated encoder is 42 counts per rev

    private static final double MOTOR_ROTATIONS_PER_INDEXER_ROTATIONS = 0; //TODO: this is a number
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

        public double getP() {
            return SmartDashboard.getNumber("P Gain", 0.001);
        }

        public double getI() {
            return SmartDashboard.getNumber("I Gan", 0.001);
        }

        public double getD() {
            return SmartDashboard.getNumber("D Gain", 0.001);
        }

        public double getFF() {
            return SmartDashboard.getNumber("Feed Forward", 0.001);
        }

        public double getIz() {
            return SmartDashboard.getNumber("I Zone", 0.001);
        }

        public double getMaxOutput() {
            return 1;
        }

        public double getMinOutput() {
            return -1;
        }

        public double getMaxRPM() {
            return 5700;
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
            return 30;
        }

        public int getPeakCurrent() {
            return 0;
        }

        public int getOffsetFromEncoderZero() {
            return 0;
        }

    }

    public static final Config INDEXER_CONFIG = new Config();

    public Indexer() {
        // TODO: find actual values for everything here. Does it make sense to require
        // all these?
        super(INDEXER_CONFIG);

        this.updraw = new TalonSRXLite(RobotMap.UPDRAW_SPINNER);

        chamberStates = new boolean[5];

        updraw.setNeutralMode(NeutralMode.Coast);
        updraw.config_kP(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_P);
        updraw.config_kI(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_I);
        updraw.config_kD(UPDRAW_CURRENT_SLOT, UPDRAW_CURR_D);
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
    
    //TODO: figure this out once we have sensor(s)
    public void fillChamber() {
        chamberStates[getBottomChamber()] = true;
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

    // TODO: check if chamber labeling direction is correct, if not make goal neg
    public void prepareToIntake() {
        double goal = getIntakeChamber() - getBottomChamber();
        if (goal < 0) {
            goal += 5;
        }
        setSmartMotionTarget(goal * MOTOR_ROTATIONS_PER_INDEXER_ROTATIONS/5.0);
        //controller.setReference(goal / 5.0, ControlType.kPosition);
    }

    public boolean isReadyToIntake() {
        return getIntakeChamber() == getBottomChamber();
    }

    // TODO: same thing with prepareToIntake: check chamber labeling direction
    public void prepareToShoot() {
        double goal = getShootChamber() - getTopChamber();
        if (goal < 0) {
            goal += 5;
        }
        setSmartMotionTarget(goal * MOTOR_ROTATIONS_PER_INDEXER_ROTATIONS/5.0);        
        //controller.setReference(goal / 5.0, ControlType.kPosition);

    }

    public boolean isReadyToShoot() {
        return getShootChamber() == getTopChamber();
    }

    /**
     * uptake into shooter
     * 
     * @param percentOutput percent output for the updraw
     * @post top chamber should be empty
     */
    public void uptake(double percentOutput) {
        updraw.set(ControlMode.PercentOutput, percentOutput);
        chamberStates[getTopChamber()] = false;
    }

    /**
     * uptake at 1.0 percentoutput
     * @post top chamber should be empty
     */
    public void uptake() {
        updraw.set(ControlMode.PercentOutput, 1.0);
        chamberStates[getTopChamber()] = false;
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

    // zeroed means that chamber 0 is at the top
    public boolean isZeroed() {
        // Probably hall sensor?
        return false;
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

    // Preferably use this instead of getDegreePos
    public double getPosition() {
        return encoder.getPosition() / RobotConstants.REVOLVER_GEAR_RATIO;
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void moveByPercentOutput(double output) {

    }

    @Override
    public double getAngleInDegrees() {
        return ((this.getPosition() / INDEXER_TICKS_PER_ROTATION) * 360);
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

}