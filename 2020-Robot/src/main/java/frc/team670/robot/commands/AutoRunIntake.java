/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import frc.team670.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.dataCollection.sensors.IRSensor;
import frc.team670.robot.utils.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * An example command that uses an example subsystem.
 */
public class AutoRunIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Intake intake;
  private IRSensor sensors;

  private boolean hasBeenTriggered, roll;
  private double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoRunIntake(Intake intake, IRSensor sensors, double speed) {
    this.intake = intake;
    this.sensors = sensors;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("current-command", "AutoRunIntake");
    hasBeenTriggered = false;
    Logger.consoleLog();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasBeenTriggered = sensors.isTriggered();
    roll = intake.isDeployed();
    if (hasBeenTriggered) {
      intake.setRolling(speed, roll);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
