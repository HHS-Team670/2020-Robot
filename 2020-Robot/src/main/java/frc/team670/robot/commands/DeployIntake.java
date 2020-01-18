package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.subsystems.Intake;

/**
 *
 */
public class DeployIntake extends InstantCommand {

  private boolean isDeploy;
  private Intake intake;
	
	/*
	 * @param isDeploy true if it has to deploy, false if it is to pick up
	 */
    public DeployIntake(boolean isDeploy, Intake intake) {
      this.isDeploy = isDeploy;
      this.intake = intake;
    }

    // Called just before this Command runs the first time and executes once
    public void initialize() {
      Logger.consoleLog("Intake deploying");
      intake.setDeploy(isDeploy);
    }

    // Called once after isFinished returns true
    public void end() {
      Logger.consoleLog();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
    	end();
    }
}

