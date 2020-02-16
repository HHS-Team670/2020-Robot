/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.team670.robot.commands.climber;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.climber.Climber;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractClimber extends CommandBase implements MustangCommand {
  private Climber climber;
  private boolean hooked;
  private boolean climbing;
  private double heightCM;

  public RetractClimber(Climber climber, double heightCM) {
    super();
    this.climber = climber;
    this.heightCM = heightCM;
    addRequirements(climber);
    hooked = false;
    climbing = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hooked) {
      hooked = climber.hookOnBar();
    } else if (!climbing) {
      climber.climb(heightCM);
      climbing = true;
    }
    // if bar is tilted, the first pull that hooks on to the bar must stop moving
    // once the second pull hooks on as well, both pulls move down
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) { // this is only if the motor is interrupted, otherwise the motor should keep the
                       // same position
      climber.set(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbing;
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return null;
  }
}