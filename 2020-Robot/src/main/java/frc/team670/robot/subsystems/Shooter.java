/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.robot.utils.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Shooter extends SubsystemBase {

  private TalonSRX shooterTalon;

  public Shooter(int CANId){
    shooterTalon = new TalonSRX(CANId); //TODO set with the constant
    SmartDashboard.putNumber("speed", 0);
    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
    SmartDashboard.putNumber("F", 0);
    setPID();
  }
  public void shoot() {
    shooterTalon.set(ControlMode.Velocity,8000);
  }

  public void off() {
    shooterTalon.set(ControlMode.PercentOutput,0);
  }

  public void setPID(){
    shooterTalon.config_kP(0, SmartDashboard.getNumber("P", 0.0));
    shooterTalon.config_kI(0, SmartDashboard.getNumber("I", 0.0));
    shooterTalon.config_kD(0, SmartDashboard.getNumber("D", 0.0));
    shooterTalon.config_kF(0, SmartDashboard.getNumber("F", 0.0));
  }
  
  public void setSpeed() {
    Logger.consoleLog("cuurent-speed %s", SmartDashboard.getNumber("speed", 0.0));
    shooterTalon.set(ControlMode.Velocity,SmartDashboard.getNumber("speed", 0.0));
    
  }

  public void periodic(){
    setPID();
    setSpeed();
  }
}
