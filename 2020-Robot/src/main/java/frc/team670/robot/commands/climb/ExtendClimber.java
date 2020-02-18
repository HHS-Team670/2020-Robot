/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb;

import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.climber.Climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendClimber extends CommandBase implements MustangCommand {

  private Climber climber;
  private double heightCM;
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public ExtendClimber(Climber climber, double heightCM) {
    this.climber = climber;
    this.heightCM = heightCM;
    addRequirements(climber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climb(heightCM);
    climber.setIsExtending(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.set(0); // TODO: check to make sure this is right
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isAtTarget();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}
