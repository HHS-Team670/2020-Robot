package frc.team670.robot.subsystems;

import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;

import frc.team670.robot.constants.RobotMap;

public class Conveyor extends MustangSubsystemBase{

    private SparkMAXLite roller;
    private CANPIDController roller_controller;
    private final double v_P = 0.01;
    private final double v_I = 0.0;
    private final double v_D = 0.0;

    private double CONVEYOR_SPEED = 0.3; // Set this to whatever target RPM if doing velocity

    public Conveyor(){
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CONVEYOR_ROLLER, Motor_Type.NEO_550);
        roller_controller = roller.getPIDController();
        roller_controller.setP(v_P);
        roller_controller.setI(v_I);
        roller_controller.setD(v_D);
    }

    public void run(){
        // roller_controller.setReference(CONVEYOR_SPEED, ControlType.kVelocity);
        roller.set(CONVEYOR_SPEED);
    }

    @Override
    public HealthState checkHealth() {
        if (roller.getLastError() != null && roller.getLastError() != CANError.kOk)
            return HealthState.RED;
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }
    
}