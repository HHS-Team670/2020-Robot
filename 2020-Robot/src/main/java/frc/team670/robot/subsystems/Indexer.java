package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.motorcontroller.TalonSRXLite;

/**
 * 
 */
public class Indexer extends SparkMaxRotatingSubsystem {

    private CANSparkMax SM;
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

    // SmartMotion control: indexer
    // TODO: find all these values
    private static final double SM_P = 0.95;
    private static final double SM_I = 0;
    private static final double SM_D = 0;
    private static final double SM_FF = 0;

    public static final int FORWARD_LIMIT = 0; // TODO set this
    public static final int REVERSE_LIMIT = 0; // TODO set this

    private static final int INDEXER_NORMAL_CONTINUOUS_CURRENT_LIMIT = 0; // TODO set this?
    private static final int INDEXER_PEAK_CURRENT_LIMIT = 0; // TODO set this?

    private static final int INDEXER_TICKS_PER_ROTATION= 42; //NEO550 integrated encoder is 42 counts per rev
    private static final int INDEXER_OFFSET_FROM_ZERO = 0;

    public Indexer() {
        super(RobotMap.INDEXER_ROTATOR, SM_P, SM_I, SM_D, SM_FF, FORWARD_LIMIT, REVERSE_LIMIT, false, INDEXER_NORMAL_CONTINUOUS_CURRENT_LIMIT, INDEXER_PEAK_CURRENT_LIMIT, INDEXER_OFFSET_FROM_ZERO);
        this.SM = rotator;
        controller = SM.getPIDController();
        this.updraw  = new TalonSRXLite(RobotMap.UPDRAW_SPINNER);
        encoder = SM.getEncoder();
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

    /**
     * called when ball goes into a chamber
     * 
     * @param chamber the one just filled
     */
    public void fillChamber(int chamber) {
        chamberStates[chamber] = true;
    }

    // chamber currently at the top
    // if its exactly between two chambers, default to later before
    public int getBottomChamber() {

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

    //TODO: check if chamber labeling direction is correct, if not change goal to bottom - intake
    public void prepareToIntake() {
        double goal = getIntakeChamber() - getBottomChamber();
        if (goal < 0) {
            goal += 5;
        }
        //setSmartMotionTarget(goal);
        controller.setReference(goal/5.0, ControlType.kPosition);
    }

    public boolean isReadyToIntake() {
        return getIntakeChamber() == getBottomChamber();
    }

    //TODO: same thing with prepareToIntake: check chamber labeling direction
    public void prepareToShoot() {
        double goal = getShootChamber() - getTopChamber();
        if (goal < 0) {
            goal += 5;
        }
        //setSmartMotionTarget(goal);
        controller.setReference(goal/5.0, ControlType.kPosition);

    }

    public boolean isReadyToShoot() {
        return getShootChamber() == getTopChamber();
    }

    public int getTopChamber() {

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
    public int getIntakeChamber() {
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

    public int getShootChamber() {
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
        SM.set(speed);
    }

    public double getSpeed() {
        return SM.get();
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
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void moveByPercentOutput(double output) {

    }

    @Override
    public double getAngleInDegrees() {
        return ((this.getPosition()/INDEXER_TICKS_PER_ROTATION)*360);
    }

}