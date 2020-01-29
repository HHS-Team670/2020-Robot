/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.team670.robot.commands.joystick_controls;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.team670.robot.subsystems.BallIndexer;
 
public class JoystickIndexer extends CommandBase {
 /**
  * Creates a new JoystickGun.
  */
  /*
  private BallIndexer ballIndexer;
  public JoystickIndexer(BallIndexer ballIndexer) {
    // Use addRequirements() here to declare subsystem dependencies.
     super();
    this.ballIndexer=ballIndexer;
    addRequirements(ballIndexer);
  }*/
 
 // Called when the command is initially scheduled.
 @Override
  public void initialize() {
    //ballIndexer.driveIndexer(0);
  }
 
 // Called every time the scheduler runs while the command is scheduled.
 @Override
  public void execute() {
    // if (RobotContainer.operatorJoystick.getTrigger()){
    //   ballIndexer.driveIndexer(RobotContainer.operatorJoystick.getY());
    // }
  }
 
 // Called once the command ends or is interrupted.
 @Override
  public void end(boolean interrupted) {
   // ballIndexer.driveIndexer(0);
  }
 // Returns true when the command should end.
 @Override
  public boolean isFinished() {
    return false;
  }
}
 
