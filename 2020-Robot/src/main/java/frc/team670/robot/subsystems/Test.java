package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;

public class Test extends SparkMaxRotatingSubsystem {

    public Test() {
        // super(RobotMap.SPARK_LEFT_MOTOR_1, SmartDashboard.getNumber("P", 0.001), 0,
        // 0, 0, 1000000, -1000000, false, 30, 5, 0);
        // display PID coefficients on SmartDashboard
        /*
            public SparkMaxRotatingSubsystem(int deviceID, int slot, double kP, 
            double kI, double kD, 
            double kFF, double kIz, double KMaxOutput,
            double kMinOutput, double kMaxRPM, double kMaxVel, 
            double kMinVel, double kMaxAcc, double kAllowedErr,
            int forwardSoftLimit, int reverseSoftLimit, boolean timeout, 
            int continuousCurrentLimit,
            int peakCurrentLimit, int offsetFromEncoderZero) {
         */
        super(RobotMap.SPARK_LEFT_MOTOR_1, 0, 
        SmartDashboard.getNumber("P", 0.001), SmartDashboard.getNumber("I", 0.001), 
        SmartDashboard.getNumber("D", 0.001), 
        SmartDashboard.getNumber("FF", 0.001), SmartDashboard.getNumber("Iz", 0.001), 
        1, -1, 
        5700, 2000, 0, 1500, 
        ALLOWED_ERR, SMARTMOTION_SLOT, SMARTMOTION_SLOT, false, 
        SMARTMOTION_SLOT, SMARTMOTION_SLOT, 0);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);

        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean("Mode", true);

    }

    @Override
    public boolean getTimeout() {

    @Override
    public void moveByPercentOutput(double output) {
        // TODO Auto-generated method stub

    }

    @Override
    public double getAngleInDegrees() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }

}