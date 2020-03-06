/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.joystickControls;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.subsystems.Turret;

public class JoystickTurret extends CommandBase implements MustangCommand {

  private Turret turret;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public JoystickTurret(Turret turret) {
    super();
    this.turret = turret;
    addRequirements(turret);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(this.turret, HealthState.YELLOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getOperatorController().getRawButton(4)) {
      double power = RobotContainer.getOperatorController().getZ()/2;
      if(Math.abs(power) > 0.025){
        turret.moveByPercentOutput(power);
      }
      else{
        turret.moveByPercentOutput(0);
      }
    }
    else{
      turret.moveByPercentOutput(0);
    }
    if (RobotContainer.getDriverController().getRawButton(10)) {
      double power = RobotContainer.getDriverController().getRightStickY()/2;
      if(Math.abs(power) > 0.025){
        turret.moveByPercentOutput(power);
      }
      else{
        turret.moveByPercentOutput(0);
      }
    }
    else{
      turret.moveByPercentOutput(0);
    }
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}