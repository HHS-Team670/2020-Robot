package frc.team670.robot.subsystems;

import com.revrobotics.ControlType;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;

public class Test extends SparkMaxRotatingSubsystem {

    public Test() {
        // display PID coefficients on SmartDashboard
        super(RobotMap.SPARK_LEFT_MOTOR_1, 0, SmartDashboard.getNumber("P", 0.001),
                SmartDashboard.getNumber("I", 0.001), SmartDashboard.getNumber("D", 0.001),
                SmartDashboard.getNumber("FF", 0.001), SmartDashboard.getNumber("Iz", 0.001), 1, -1, 5700, 2000, 0,
                1500, 50, 1000, -1000, false, 30, 0, 0);

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", MAX_OUTPUT);
        SmartDashboard.putNumber("Min Output", MIN_OUTPUT);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", MAX_VEL);
        SmartDashboard.putNumber("Min Velocity", MIN_VEL);
        SmartDashboard.putNumber("Max Acceleration", MAX_ACC);
        SmartDashboard.putNumber("Allowed Closed Loop Error", ALLOWED_ERR);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);

    }

    public void valueTest() {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            controller.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            controller.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            controller.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            controller.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            controller.setFF(ff);
            kFF = ff;
        }
        if ((max != MAX_OUTPUT) || (min != MIN_OUTPUT)) {
            controller.setOutputRange(min, max);
            MIN_OUTPUT = min;
            MAX_OUTPUT = max;
        }
        if ((maxV != MAX_VEL)) {
            controller.setSmartMotionMaxVelocity(maxV, 0);
            MAX_VEL = maxV;
        }
        if ((minV != MIN_VEL)) {
            controller.setSmartMotionMinOutputVelocity(minV, 0);
            MIN_VEL = minV;
        }
        if ((maxA != MAX_ACC)) {
            controller.setSmartMotionMaxAccel(maxA, 0);
            MAX_ACC = maxA;
        }
        if ((allE != ALLOWED_ERR)) {
            controller.setSmartMotionAllowedClosedLoopError(allE, 0);
            ALLOWED_ERR = allE;
        }

        double setPoint, processVariable;
        setPoint = SmartDashboard.getNumber("Set Position", 0);
        /**
         * As with other PID modes, Smart Motion is set by calling the setReference
         * method on an existing pid object and setting the control type to kSmartMotion
         */
        // Methods from SMRS below
        setSmartMotionTarget(setPoint);
        processVariable = getUnadjustedPosition();

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("Process Variable", processVariable);
        SmartDashboard.putNumber("Output", rotator.getAppliedOutput());
    }

    @Override
    public boolean getTimeout() {
        return false;
    }

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