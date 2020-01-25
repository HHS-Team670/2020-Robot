/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.joystick_controls;
import frc.team670.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickTurret extends CommandBase {
  /**
   * Creates a new JoystickTurret.
   */
  private Turret turret;
  public JoystickTurret(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    super();
    this.turret=turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.//method, initialize 0
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    float hyp= Math.sqrt(Math.pow(RobotContainer.operatorJoystick.getX(), 2) + Math.pow(RobotContainer.operatorJoystick.getY(), 2));
    turret.//method, call hyp
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.//method, end 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
