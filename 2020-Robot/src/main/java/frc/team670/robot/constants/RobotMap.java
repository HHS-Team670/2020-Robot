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
  public final static int SHOOTER_ID_MAIN = 11;
  public final static int SHOOTER_ID_FOLLOWER = 12;

  // Motors
  public static final int COLOR_WHEEL_MOTOR_ID = 10;
  // PC Module
  public static final int PCMODULE = 2;

  // Intake
  public static final int INTAKE_ROLLER = 4;

  // Conveyor
  public static final int CONVEYOR_ROLLER = 8;

  //Indexer and Updraw
  public static final int INDEXER_ROTATOR = 9;
  public static final int UPDRAW_SPINNER = 10;

}