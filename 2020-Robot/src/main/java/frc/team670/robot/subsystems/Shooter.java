/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Shooter extends SubsystemBase {

  private TalonSRX shooterTalon;

  public Shooter(int CANId){
    shooterTalon = new TalonSRX(CANId); //TODO set with the constant
    SmartDashboard.putNumber("speed", 0);
  }
  public void shoot() {
    shooterTalon.set(ControlMode.PercentOutput,0.5);
  }

  public void off() {
    shooterTalon.set(ControlMode.PercentOutput,0);
  }
  
  public void setSpeed() {
    shooterTalon.set(ControlMode.PercentOutput,SmartDashboard.getNumber("speed", 0));
    
  }

  public void periodic(){
    setSpeed();
  }
}
