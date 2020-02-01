/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.joystick_controls;
import frc.team670.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.subsystems.Turret;

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

    turret.rotateToAngle(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = Math.atan((RobotContainer.operatorJoystick.getY())/(RobotContainer.operatorJoystick.getX()));
    turret.rotateToAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.rotateToAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}