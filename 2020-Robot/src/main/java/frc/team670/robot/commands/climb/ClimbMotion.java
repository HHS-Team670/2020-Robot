/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.team670.robot.commands.climb;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Brings the robot up and off the ground by retracting the climber pull.
 */
public class ClimbMotion extends CommandBase implements MustangCommand {
  private Climber climber;
  private static final double MAX_CLIMBING_DISTANCE_CM = 1; // Estimate 2/27: Height 114cm, min bar height 128 cm, 4cm
                                                             // allow
  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public ClimbMotion(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MustangScheduler.getInstance().schedule(new ExtendClimber(climber));
    MustangScheduler.getInstance().schedule(new HookOnBar(climber));
    MustangScheduler.getInstance().schedule(new Climb(climber));
  }


  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}