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
  /**
   * Creates a new ExampleSubsystem.
   */

  private TalonSRX shooterTalon;

  public Shooter(){
    shooterTalon = new TalonSRX(1);
    SmartDashboard.putNumber("speed", 0);
  }

	public void shoot() {
		shooterTalon.set(ControlMode.Velocity,1);
  }
  
	public void setSpeed(double speed) {
		shooterTalon.set(ControlMode.Velocity,SmartDashboard.getNumber("speed", 0));
  }

  
  
  

}
