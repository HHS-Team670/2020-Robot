package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.IRSensor;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.MotorConfig;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

    private SparkMAXLite roller;
    private int currentLimit;
    private boolean deployed;
    
    public Intake() {

        roller = new SparkMAXLite(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        roller.setSmartCurrentLimit(currentLimit);
        currentLimit = 20;

        //For testing, just assume it's deployed
        deployed = true;

    }
    public boolean isRolling() {
        return roller.get() != 0;
    }


    public void roll(double speed) {
        roller.set(speed);
    }


    @Override
    public HealthState checkHealth() {
        if (deployed) {
            return HealthState.GREEN;
        } else {
            return HealthState.RED;
        }
    }

    @Override
    public void mustangPeriodic() {
    }
}