package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.subsystems.Intake;

import frc.team670.robot.subsystems.Intake;

/**
 * @author Khicken
 */
public class DeployIntake extends InstantCommand {

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
      intake.intake.setDeploy(isDeploy);
      Logger.consoleLog("Intake deployed");
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

    public void a_deploy() { // uses ir sensor to detect ball to deploy then roll/spin motorz(autonomous deploy)
        if (!intake.isDeployed() && intake.getSensor()) {
            intake.setDeploy(true);
            if (isDeployed()) {
                setRolling(pValue, true);
            }
        }
    }

    public void m_deploy() {
        if (!intake.isDeployed()) {
            intake.setDeploy(true);
            if(intake.isDeployed()) {
                setRolling(pValue, true);
            }
        }
    }

    public void retract() {
        intake.fr_retract();
    }

    public void unjam() { // use if thing jammed 
        if(!intake.isDeployed()) {
            intake.setDeploy(true);
        }

        setRolling(-pValue, true);

        // TODO 
    }



}

