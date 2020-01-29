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
    

    public Indexer(CANSparkMax SM) {
        this.SM = SM;
        encoder = SM.getEncoder();
        totalNumBalls = 0;
    }

    public int totalNumOfBalls() {
        return totalNumBalls;
    }

    public void setNumBalls(int num) {
        totalNumBalls = num;
    }

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