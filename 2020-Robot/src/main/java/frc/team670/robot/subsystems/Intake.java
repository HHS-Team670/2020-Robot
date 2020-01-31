package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.IRSensor;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

    private Compressor comp;
	private Solenoid deployer;
    private CANSparkMax roller;
    private CANPIDController pidController;
    private CANEncoder encoder;
    private IRSensor sensor;

    private boolean isDeployed, isRolling;
    private double rollerSpeed;
    public double kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput, maxRPM;
    public Intake() {
        /* INITIALIZE HARDWARE COMPONENTS */
        comp = new Compressor(RobotMap.PCMODULE); // need to edit RobotMap to update ports for irsensor and other stuff
        comp.setClosedLoopControl(true);
        //deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_SOLENOID); //define once RobotMap ID given
        roller = new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        pidController = roller.getPIDController();
        encoder = roller.getEncoder();
        // sensor = new IRSensor(RobotMap.INTAKE_IRSENSOR);

        /* SOME RANDOM VARIABLE(S) */
        

        /* PID STUFFS */
        // PID coefficients
        kP = 0;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMinOutput = -1;
        kMaxOutput = 1;
        maxRPM = 0;

        // Set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
    }

    /* helper and basic fetch methods */
    public boolean getSensor() {
        return sensor.isTriggered();
    }

    public boolean isRolling() {
        return isRolling;
    }
    
    public boolean isDeployed() {
        return isDeployed;
    }

    /* setting methods */
    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }

    public void setRolling(boolean roll) {
        isRolling = roll;
        if(isRolling)
            roller.set(rollerSpeed);
        else
            roller.set(0);
    }

    public void setRollerSpeed(double percent) {
        rollerSpeed = percent;
    }

    @Override
    public HealthState checkHealth() {
        if(/*encoder.getCountsPerRevolution() != (kP + kI + kD)*/isDeployed() != true) { return HealthState.RED;} // if not deployed, UH OH STINKY
        else return HealthState.GREEN;
    }

    public void teleopPeriodic() {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        if(p != kP) {
            kP = p;
            pidController.setP(kP);
        }
        if(i != kI) {
            kI = i;
            pidController.setI(kI);
        }
        if(d != kD) {
            kD = d;
            pidController.setD(kD);
        }
        if(iz != kIz) {
            kIz = iz;
            pidController.setIZone(kIz);
        }
        if(ff != kFF) {
            kFF = ff;
            pidController.setFF(kFF);
        }
        if(min != kMinOutput || max != kMaxOutput) {
            kMinOutput = min;
            kMaxOutput = max;
            pidController.setOutputRange(kMinOutput, kMaxOutput);
        }

        double setPoint = maxRPM;
        pidController.setReference(setPoint, ControlType.kVelocity);

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());
    }
}