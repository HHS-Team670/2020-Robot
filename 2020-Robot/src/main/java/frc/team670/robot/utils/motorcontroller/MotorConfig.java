package frc.team670.robot.utils.motorcontroller;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MotorConfig {

    public enum BrushlessMotor {

        NEO(40), NEO_550(25);

        int MAX_CURRENT;

        BrushlessMotor(int currentLimit) {
            MAX_CURRENT = currentLimit;
        }

        public int getMaxCurrent() {
            return MAX_CURRENT;
        }
        
    }

    public enum DCMotor {

        REDLINE_775(40), BAG(30);

        int MAX_CURRENT;

        DCMotor(int currentLimit) {
            MAX_CURRENT = currentLimit;
        }

        public int getMaxCurrent() {
            return MAX_CURRENT;
        }

    }

}