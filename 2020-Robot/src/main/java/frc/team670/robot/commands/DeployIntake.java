package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.subsystems.Intake;

/**
 * @author Khicken
 */
public class DeployIntake extends InstantCommand implements MustangCommandBase {

    private boolean isDeploy;
    private Intake intake;

	/*
	 * @param isDeploy true if it is to deploy, false if it is to pick up
	 */
    public DeployIntake(boolean isDeploy, Intake intake) {
      this.isDeploy = isDeploy;
      this.intake = intake;
    }

    // Called just before this Command runs the first time and executes once
    public void initialize() {
      Logger.consoleLog("Intake deploying");
      intake.setDeploy(isDeploy);
      if(intake.isDeployed()) {
        Logger.consoleLog("Intake deployed");
      } else {
          intake.checkHealth(); // oh no! some error stuffs
      }
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

    /* INTAKE COMMANDSSSSSS */

    public void deploy() { // yes
        if(!intake.isDeployed()) intake.setDeploy(true); // deploy intake if not

        intake.setRollerSpeed(intake.kP + intake.kI + intake.kD); // set pid roller sped
        intake.setRolling(true); // stonks
    }

    public void retract() { // no
        if(intake.isDeployed()) intake.setDeploy(false); // move intake back

        intake.setRolling(false); // stop rolling motors
    }
}

