package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap{
  
  public static final int PDP_ID = 0;

  public static final int DRIVER_CONTROLLER_PORT=0;
  public static final int OPERATOR_CONTROLLER_PORT=1;

  // Drive Base
  public static final int SPARK_LEFT_MOTOR_1 = 20; // These are properly set. 
  public static final int SPARK_LEFT_MOTOR_2 = 21; 
  public static final int SPARK_RIGHT_MOTOR_1 = 22;
  public static final int SPARK_RIGHT_MOTOR_2 = 23;

  // NavX
  public final static Port NAVX_PORT = SerialPort.Port.kUSB;  

  //PCModule
  public final static int PCMODULE = 2;

  //Intake
  public final static int INTAKE_SOLENOID = 0;
  public final static int INTAKE_ROLLERS = 4;
  public final static int INTAKE_IRSENSOR = 5;

}