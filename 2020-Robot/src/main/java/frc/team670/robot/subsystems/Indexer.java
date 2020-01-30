package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Indexer extends MustangSubsystemBase{

    private CANSparkMax SM;
    private CANEncoder encoder;
    private int totalNumBalls;
    private boolean[] chamberStates;

    public Indexer(CANSparkMax SM) {
        this.SM = SM;
        encoder = SM.getEncoder();
        totalNumBalls = 0;
        chamberStates = new boolean[5];
    }

    public int totalNumOfBalls() {
        return totalNumBalls;
    }

    public void setNumBalls(int num) {
        totalNumBalls = num;
    }

    /**
     * 
     * @param chamber the chamber to check
     * @return if its full
     */
    public boolean checkChamber(int chamber) {
        return chamberStates[chamber];
    }

    /**
     * checks collective state of all chambers
     * @return true if there's a ball in every chamger
     */
    public boolean isFull() {
        for (boolean chamber: chamberStates)
            if (!chamber)
                return false;

        return true;
    }

    /**
     * logic for rotatetoshoot
     * @return the first full chamber en route to shoot
     */
    public int getFirstFull() {
        if (isEmpty())
            return -1;

        if (chamberStates[getCurrentChamber()])
            return getCurrentChamber();

        int c = getCurrentChamber() - 1;

        if (getCurrentChamber() == 0)
            c = 4;        

        while (c >= 0) {
            if(chamberStates[c])
                return c;
            c--;
        }

        return -1; // should not be reached
    }

    /**
     * more logic for rotatetointake
     * @return the last full chamber en route to shoot
     */
    public int getLastFull() {
        if (isEmpty())
            return -1;
        if (!chamberStates[getCurrentIntakeChamber()])
            return getCurrentIntakeChamber();
        int c = getCurrentIntakeChamber() - 1;

        if (getCurrentIntakeChamber() == 0)
            c = 4;        

        while (c >= 0) {
            if(!chamberStates[c])
                return c;
            c--;
        }

        return -1; // should not be reached
    }

    /**
     * checks collective state of all chambers
     * @return true if there's no ball in any chambers
     */
    public boolean isEmpty() {
        for (boolean chamber: chamberStates)
            if (chamber)
                return false;

        return true;
    }

    /**
     * called when ball goes into a chamber
     * @param chamber the one just filled
     */
    public void fillChamber(int chamber) {
        chamberStates[chamber] = true;
    }

    // chamber currently at the top
    // if its exactly between two chamber, default to later before
    public int getCurrentChamber() {
        double degrees = getDegreePos();
        if (degrees <= 36 && degrees > 324)
            return 0;
        else if (degrees <= 108 && degrees > 36)
            return 1;
        else if (degrees <= 180 && degrees > 108)
            return 2;
        else if (degrees <= 252 && degrees > 180)
            return 3;
        else
            return 4;
    }

    public int getCurrentIntakeChamber() {
        double degrees = getDegreePos();
        if (degrees <= 144 && degrees > 216)
            return 0;
        else if (degrees <= 72 && degrees > 144)
            return 1;
        else if (degrees <= 0 && degrees > 172)
            return 2;
        else if (degrees <= 216 && degrees > 0)
            return 3;
        else
            return 4;
    }

    // zeroed means that chamber 0 is at the top
    public boolean isZeroed() {
        //Probably hall sensor?
        return false;
    }

    public boolean ballIn() {
        //FSR maybe?
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


    public double getPosition() {
        return encoder.getPosition();
    }

    // TODO: figure this out
    // assume degrees = 0 is zeroed
    // zeroed means chamber 0 is at the top
    public double getDegreePos() {
        return 0;
    }

    /**
     * -1 for negative direction, 1 for positive direction
     */
    public int directionToTurn () {
        if (encoder.getPosition() % 1.0 < 0.5) {
            return -1;
        } else {
            return 1;
        }
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }




}