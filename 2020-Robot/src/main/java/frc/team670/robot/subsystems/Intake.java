package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.IRSensor;

import java.awt.geom.Point2D.Double;

import com.ctre.phoenix.motorcontrol.NeutralMode;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
/*
 * @author kaleb
*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends BaseIntake {
    private Compressor comp;
    private Solenoid deployer;
    private CANSparkMax roller;
    private IRSensor sensor;
    private boolean isDeployed, isRolling;
    private double rollingSpeed;

    public static final double DISTANCE_FROM_ARM_ZERO = 28;

    private static final int ROLLER_CONTINUOUS_CURRENT = 30, ROLLER_PEAK_CURRENT = 40;
  
    public static final int INTAKE_ANGLE_IN = -90, INTAKE_ANGLE_DEPLOYED = 90;
    public static final double INTAKE_FIXED_LENGTH_IN_INCHES = 11.25, INTAKE_ROTATING_LENGTH_IN_INCHES = 14;
    private static final double MAX_BASE_OUTPUT = 0.75;
    private static final double kF = 0, kP = 0.45, kI = 0, kD = 0;
  
    // Motion Magic
    private static final int kPIDLoopIdx = 0, MOTION_MAGIC_SLOT = 0, kTimeoutMs = 0;
    private static final int OFFSET_FROM_ENCODER_ZERO = 426;
    private static final int FORWARD_SOFT_LIMIT = 932, REVERSE_SOFT_LIMIT = -979;
    private static final int CONTINUOUS_CURRENT_LIMIT = 20, PEAK_CURRENT_LIMIT = 0;
    private final static int INTAKE_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 200,  INTAKE_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_SECOND = 1200;
    private static final int QUAD_ENCODER_MAX = FORWARD_SOFT_LIMIT + 300, QUAD_ENCODER_MIN = REVERSE_SOFT_LIMIT - 300;
  
    private static final double ARBITRARY_FEED_FORWARD = 0.175;
  
    private static final int TICKS_PER_ROTATION = 4096;
  
    public static final int INTAKE_RUNNING_CURRENT = 8;
  
    // Roller
    private static final int ROLLER_CURRENT_SLOT = 1;
    private static final double ROLLER_CURR_P = 0.2, ROLLER_CURR_I = 0.0, ROLLER_CURR_D = 0.0;
    
    private Point2D.Double intakeCoord;
  
    public static final double RUNNING_POWER = 0.35;
    public static final double PICKUP_RUNNING_POWER = 0.1;

    private double changingRollingSpeed = 0.8;

    public Intake() {
        super(new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless), ARBITRARY_FEED_FORWARD, FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT, true, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, OFFSET_FROM_ENCODER_ZERO);
        comp = new Compressor(RobotMap.PCMODULE);
        comp.setClosedLoopControl(true);
        deployer = new Solenoid(RobotMap.PCMODULE, RobotMap.INTAKE_DEPLOYER);
        roller = new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        sensor = new IRSensor(RobotMap.INTAKE_SENSOR);
    }

    // basic fetch and set commands
    public void setDeploy(boolean dep) {
        isDeployed = dep;
        deployer.set(isDeployed);
    }

    public void setRolling(double speed, boolean roll) {
        isRolling = roll;
        rollingSpeed = speed;
        if (isRolling) {
            // roller.set(ControlMode.PercentOutput, rollingSpeed);
            roller.set(rollingSpeed);
        } else {
            roller.set(0);
        }
    }

    public void setRollingSpeed(double speed) {
        rollingSpeed = speed;
    }

    public boolean getSensor() {
        return sensor.isTriggered();
    }

    public boolean isDeployed() {
        return isDeployed;
    }

    public boolean isRolling() {
        return isRolling;
    }

    // deploy, retract, and roll commands(speeds need to be set)
    public void a_deploy() { // uses ir sensor to detect ball to deploy then roll/spin motorz(autonomous
                             // deploy)
        if (!isDeployed() && getSensor()) {
            setDeploy(true);
            if (isDeployed()) {
                setRolling(changingRollingSpeed, true);
            }
        }
    }

    public void m_deploy() {
        if (!isDeployed()) {
            setDeploy(true);
            if (isDeployed()) {
                setRolling(changingRollingSpeed, true);
            }
        }
    }

    public void retract() {
        setDeploy(true);
        if (isDeployed()) {
            setRolling(changingRollingSpeed, true);
        }
    }

    public void unjam() { // use if thing jammed 
        if (!isDeployed()) {
            setDeploy(true);
        }

        setRolling(-changingRollingSpeed, true);

        // TODO 
    }

    // base system(fetch methods and stuff)
    private static int convertIntakeDegreesToTicks(double degrees) {
        //If straight up is 0 and going forward is positive
        // percentage * half rotation
        return (int)((degrees / 360) * TICKS_PER_ROTATION);
      }
    
      /**
       * Converts intake ticks into an angle
       */
      private static double convertIntakeTicksToDegrees(double ticks) {
        //If straight up is 0 and going forward is positive
        return ((360 * ticks) / TICKS_PER_ROTATION);
      }
    
    @Override
    protected double getArbitraryFeedForwardAngleMultiplier() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getAngleInDegrees() {
        return convertIntakeTicksToDegrees(getPositionTicks());
    }

    @Override
    public Double getMotionMagicDestinationCoordinates() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setRotatorNeutralMode(NeutralMode mode) {
        // TODO Auto-generated method stub

    }

    @Override
    public Double getIntakeCoordinates() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void runIntake(double power, boolean runningIn) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setMotionMagicSetpointAngle(double angle) {
        // TODO Auto-generated method stub

    }
}