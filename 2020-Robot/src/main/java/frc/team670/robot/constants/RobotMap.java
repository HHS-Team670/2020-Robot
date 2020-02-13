package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap {

  public static final int PDP_ID = 0;

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // Drive Base
  public static final int SPARK_LEFT_MOTOR_1 = 20; // These are properly set.
  public static final int SPARK_LEFT_MOTOR_2 = 21;
  public static final int SPARK_RIGHT_MOTOR_1 = 22;
  public static final int SPARK_RIGHT_MOTOR_2 = 23;
  
  // NavX
  public final static Port NAVX_PORT = SerialPort.Port.kUSB;

  // Shooter
  public final static int SHOOTER_STAGE_2_MAIN = 11;
  public final static int SHOOTER_STAGE_2_FOLLOWER = 12;
  public static final int SHOOTER_STAGE_1 = 13;

  // Climber
  public static final int CLIMBER_MOTOR_ID_1 = 14;
  public static final int CLIMBER_MOTOR_ID_2 = 15;
  
   // PC Module
   public static final int PCMODULE = 2;

  //Turret 
  public static final int TURRET_ROTATOR = 24;

  // Color Wheel
  public static final int COLOR_WHEEL_MOTOR_ID = 7;

  // Intake
  public static final int INTAKE_ROLLER = 4;
  public static final int INTAKE_DEPLOYER = 3;    //SET THIS
  public static final int INTAKE_COMPRESSOR = 6;  //SET THIS

  // Conveyor
  public static final int CONVEYOR_ROLLER = 8;

  //Indexer and Updraw
  public static final int INDEXER_ROTATOR = 9;
  public static final int UPDRAW_SPINNER = 10;

}
