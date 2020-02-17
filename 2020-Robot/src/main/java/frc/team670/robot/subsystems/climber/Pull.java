package frc.team670.robot.subsystems.climber;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

/**
 * @author Pallavi & Eugenia
 */
public class Pull extends MustangSubsystemBase {

    // TODO: define constants
    private static final double PULL_P = 0;
    private static final double PULL_I = 0;
    private static final double PULL_D = 0;
    private static final double PULL_FF = 0;
    private static final double NORMAL_OUTPUT = 0;
    private static final double ROTATIONS_PER_CM = 0;
    private static final double HALF_CM = 0.5 / ROTATIONS_PER_CM;

    private CANPIDController controller;
    private CANEncoder encoder;
    private SparkMAXLite motor;
    private boolean hookedOnBar;
    private int inversionFactor;
    private double target;
    private Solenoid solenoid;

    public Pull(boolean inverted) {
        if (inverted) {
            motor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_RIGHT_MOTOR, Motor_Type.NEO);
            inversionFactor = -1;
            solenoid = new Solenoid(RobotMap.PCMODULE, RobotMap.CLIMBER_RIGHT_SOLENOID);

        } else {
            motor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_LEFT_MOTOR, Motor_Type.NEO);
            inversionFactor = 1;
            solenoid = new Solenoid(RobotMap.PCMODULE, RobotMap.CLIMBER_LEFT_SOLENOID);

        }
        controller = motor.getPIDController();
        encoder = motor.getEncoder();
        setDefaultPID();
    }

    public void setDefaultPID() {
        controller.setP(PULL_P);
        controller.setI(PULL_I);
        controller.setD(PULL_D);
        controller.setFF(PULL_FF);
    }

    public void solenoidOff() {
        solenoid.set(false);
    }

    public void solenoidOn() {
        solenoid.set(true);
    }

    public boolean isHookedOnBar() {
        if (motor.getOutputCurrent() > NORMAL_OUTPUT && !hookedOnBar) {
            setPower(0);
            hookedOnBar = true;
        }

        if (!hookedOnBar) {
            setPower(-0.5);
        }
        return hookedOnBar;
    }

    public void setPower(double power) {
        motor.set(power * inversionFactor);
    }

    public void climb(double heightCM) {
        double rotations = heightCM / ROTATIONS_PER_CM * inversionFactor;
        target = rotations;
        controller.setReference(rotations, ControlType.kPosition);
    }

    @Override
    public HealthState checkHealth() {
        if (isSparkMaxErrored(motor) || solenoid == null) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    public boolean isAtTarget() {
        return Math.abs(encoder.getPosition() - target) < HALF_CM;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

}