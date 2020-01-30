/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.utils.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Shooter extends MustangSubsystemBase {

   //Practice Values
   public static final double kP =0.24;
   public static final double kI =0.0;
   public static final double kD =9.670;
   public static final double kF =0.033818;

  private TalonSRX shooterTalonMain;
  private TalonSRX shooterTalonFollower;

  public Shooter(int mainCANId, int followerCANId){
    shooterTalonMain = new TalonSRX(mainCANId);
    shooterTalonFollower.set(ControlMode.Follower, followerCANId);
    SmartDashboard.putNumber("speed", 0);
    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
    SmartDashboard.putNumber("F", 0);
    setPID();
  }

  public void stop() {
    shooterTalonMain.set(ControlMode.PercentOutput,0);
  }

  public void setDefaultPID(){
    shooterTalonMain.config_kP(0, kP);
    shooterTalonMain.config_kI(0, kI);
    shooterTalonMain.config_kD(0, kD);
    shooterTalonMain.config_kF(0, kF);
  }

  public void setPID(){
    shooterTalonMain.config_kP(0, SmartDashboard.getNumber("P", 0.0));
    shooterTalonMain.config_kI(0, SmartDashboard.getNumber("I", 0.0));
    shooterTalonMain.config_kD(0, SmartDashboard.getNumber("D", 0.0));
    shooterTalonMain.config_kF(0, SmartDashboard.getNumber("F", 0.0));
  }
  
  public void setSpeed() {
    Logger.consoleLog("cuurent-speed %s", SmartDashboard.getNumber("speed", 0.0));
    shooterTalonMain.set(ControlMode.Velocity,SmartDashboard.getNumber("speed", 0.0));
    
  }

  public void periodic(){
    setPID();
    setSpeed();
  }

  @Override
  public HealthState checkHealth() {
    // TODO Auto-generated method stub
    return null;
  }
}