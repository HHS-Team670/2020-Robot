package frc.team670.robot.subsystems.climber;
 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
 
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.constants.RobotMap;
 
public class Pull extends MustangSubsystemBase {
 
    //TODO: define constants
    private static final double PULL_P= 0;
    private static final double PULL_I= 0;
    private static final double PULL_D= 0;
    private static final double PULL_FF= 0;
    private static final double NORMAL_OUTPUT = 0; 
    private static final double ROTATIONS_PER_CM = 0;
    private static final double HALF_CM = 0.5/ROTATIONS_PER_CM;
 
    private CANPIDController controller;
    private CANEncoder encoder;
    private CANSparkMax motor;
    private boolean hookedOnBar;
    private int inversionFactor;
    private double target;
 
    public Pull(boolean inverted) {
        if(inverted) {
            motor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER__MOTOR); //TODO: set motor id + decide which motor is inverted
            inversionFactor = -1;
        } else {
            motor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER__MOTOR); //TODO: set motor id + decide which motor is inverted
            inversionFactor = 1;
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
 
    public boolean hookOnBar() {
          if(motor.getOutputCurrent() > NORMAL_OUTPUT && !hookedOnBar) {
            setPower(0);
            hookedOnBar = true;
          }
 
          if(!hookedOnBar) {
            setPower(-0.5);
          }
          return hookedOnBar;
    }
 
    public void setPower(double power) {
        motor.set(power*inversionFactor);
    }
 
    public void climb(double heightCM) {
        encoder.setPosition(0);
        double rotations = heightCM/ROTATIONS_PER_CM*inversionFactor;
        target = rotations;
        controller.setReference(rotations, ControlType.kPosition);
    }
 
    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }
 
    public boolean isAtTarget() {
        return Math.abs(encoder.getPosition() - target) < HALF_CM;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }
 
}
