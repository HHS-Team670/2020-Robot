package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.dataCollection.sensors.BeamBreak;
import frc.team670.robot.utils.Logger;

/**
 * 
 * @author el
 */
public class Indexer extends SubsystemBase{


    private BeamBreak[] sensors;
    private CANSparkMax SM;
    private CANEncoder encoder;
    private double speed;
    private boolean sendingBalls;

    public Indexer(BeamBreak BB1, BeamBreak BB2, BeamBreak BB3, BeamBreak BB4, BeamBreak BB5, CANSparkMax SM) {
        sensors = new BeamBreak[5];
        sensors[0] = BB1;
        sensors[1] = BB2;
        sensors[2] = BB3;
        sensors[3] = BB4;
        sensors[4]= BB5;
        this.SM = SM;
        encoder = SM.getEncoder();
        speed = 0.9;
        sendingBalls = false;
        //Logger.consoleLog("Encoder rotations upon instantiation (should be zero): " + encoder.getPosition());
    }

    public int totalNumOfBalls() {
        int totalNum = 0;
        for (BeamBreak BB: sensors) {
            if (BB.isTriggered()) {
                totalNum++;
            }
        }
        return totalNum;
    }


    /**
     * Returns array of booleans length 5, indexes correspond to order of sensors passed in, true means there's a ball in that sensor
     */
    public boolean[] positionOfBalls() {
        boolean[] positions = new boolean[5];
        for (int i = 0; i < sensors.length; i++) {
            if (sensors[i].isTriggered()) {
                positions[i] = true;
            }
        }
        return positions;
    }

    public void sendAllToShooter() {
        if (totalNumOfBalls() != 0) {

            SM.set(speed);
            sendingBalls = true;
            
        } else {
            Logger.consoleLog("Tried to send balls to shooter, zero balls");
        }
    }

    public void periodic() {

        if (sendingBalls) {
            boolean stillHaveBallsToSend = false;
            for (int i = 1; i < sensors.length; i++) {
                if (sensors[i].isTriggered()) {
                    stillHaveBallsToSend = true;
                }
            }

            if (!stillHaveBallsToSend) {
                double errorOffset = 0.95;
                if (encoder.getPosition() % 1 > errorOffset) {
                    SM.set(0);
                }
            }

        }

    }


}