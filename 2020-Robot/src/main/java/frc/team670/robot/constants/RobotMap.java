package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap{
    public static final int PDP_ID = 0;

    public static final int DRIVER_CONTROLLER_PORT=0;

  // Drive Base
  public static final int SPARK_LEFT_MOTOR_1 = 20; // These are properly set. 
  public static final int SPARK_LEFT_MOTOR_2 = 21; 
  public static final int SPARK_RIGHT_MOTOR_1 = 22;
  public static final int SPARK_RIGHT_MOTOR_2 = 23;

  //Encoders
      //Drivebase
  public static final int LEFT_ENCODER_CHANNEL_A = 0; // These are properly set
  public static final int LEFT_ENCODER_CHANNEL_B = 1;
  public static final int RIGHT_ENCODER_CHANNEL_A = 2;
  public static final int RIGHT_ENCODER_CHANNEL_B = 3;

  public final static Port NAVX_PORT = SerialPort.Port.kUSB;  

  // Motors
  public static final int COLOR_WHEEL_MOTOR_CAN_ID = 10;

}