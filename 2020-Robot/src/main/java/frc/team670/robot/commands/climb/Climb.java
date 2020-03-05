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
import frc.team670.robot.subsystems.Climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Brings the robot up and off the ground by retracting the climber pull.
 */
public class Climb extends CommandBase implements MustangCommand {
  private Climber climber;
  private static final double MAX_CLIMBING_DISTANCE_CM = 1; // Estimate 2/27: Height 114cm, min bar height 128 cm, 4cm
                                                             // allow
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public Climb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(climber, HealthState.GREEN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setExtending(false);
    climber.setPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climb(MAX_CLIMBING_DISTANCE_CM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setPower(0);
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