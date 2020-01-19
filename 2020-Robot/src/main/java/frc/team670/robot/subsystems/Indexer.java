package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.dataCollection.sensors.BeamBreak;
import frc.team670.robot.utils.Logger;

/**
 * 
 * @author el
 */
public class Indexer extends SubsystemBase{

    private BeamBreak frontSensor, backSensor;
    private CANSparkMax SM;
    private CANEncoder encoder;
    private double speed;
    private int totalNumBalls;
    private boolean lastSensorCheck, rotating, sending, fixing;
    private double rotationGoal;

    public Indexer(BeamBreak frontSensor, BeamBreak backSensor, CANSparkMax SM) {
        this.frontSensor = frontSensor;
        this.backSensor = backSensor;
        this.SM = SM;
        encoder = SM.getEncoder();
        speed = 0.9;
        totalNumBalls = 0;
        sending = false;
        lastSensorCheck = false;
        fixing = false;
        rotationGoal = 0;
    }

    public int totalNumOfBalls() {
        return totalNumBalls;
    }

    public void periodic() {

        if (frontSensor.isTriggered()) {
            lastSensorCheck = true;
        } else {
            if (lastSensorCheck == true) {
                totalNumBalls++;
                rotateOneCompartment();
            }
        }

        if (rotating) {
            SM.set(speed);
            if (encoder.getPosition() > rotationGoal - 0.01) {
                rotating = false;
                SM.set(0);
                
            }
        }

        if (sending) {
            SM.set(speed);

 
            if (encoder.getPosition() > rotationGoal - 0.01) {
                sending = false;
                SM.set(0);


                
                //Means the rotation has been offset too much and needs to be reset
                if (encoder.getPosition() % 0.2 < 0.17 && encoder.getPosition() % 0.2 > 0.3) {
                    fixing = true;
                }
            }
        }

        if (fixing) {
            SM.set(0.5);
            if (encoder.getPosition() % 0.2 > 0.19 || encoder.getPosition() % 2 < 0.1) {
                fixing = false;
                SM.set(0);
            }
        }

    }

    private void rotateOneCompartment() {
        rotating = true;
        rotationGoal = encoder.getPosition() + 0.2;
    }


    /**
     * Must also activate the conveyor belt when calling this method
     */
    public void sendAllToShooter() {
        sending = true;
        rotationGoal = encoder.getPosition() + 1;
    }


}