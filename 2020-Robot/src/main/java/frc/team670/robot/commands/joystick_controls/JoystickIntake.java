/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.team670.robot.commands.joystick_controls;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.subsystems.Intake;
 
 
public class JoystickIntake extends CommandBase {
 /**
  * Creates a new Joystick_Intake.
  */
 
 private Intake intake;
 public JoystickIntake(Intake intake) {
   // Use addRequirements() here to declare subsystem dependencies.
   this.intake = intake;
   addRequirements(intake);
  
 }
 
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
   //intake.driveIntake(0);
 }
 
 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
   if (RobotContainer.operatorJoystick.getTop()){
     //intake.driveIntake(RobotContainer.operatorJoystick.getY());
   }
 }
 
 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
   //intake.driveInake(0);
 }
 
 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   return false;
 }
}
