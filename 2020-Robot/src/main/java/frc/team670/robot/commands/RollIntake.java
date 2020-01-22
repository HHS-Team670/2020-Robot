package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.utils.Logger;

/**
 *
 */
public class RollIntake extends InstantCommand {

	private double speed;
	private Intake intake;
	private boolean isDeployed, roll;

	public RollIntake(double speed, Intake intake) {
		this.speed = speed;
		this.intake = intake;
		addRequirements(intake);
    }

    // Called just before this Command runs the first time and executes once
	public void initialize() {
		isDeployed = intake.isDeployed();
		if (isDeployed) {
			roll = true;
		}
		intake.setRolling(speed, roll);
		Logger.consoleLog("Speed", speed);
	}

    // Called once after isFinished returns true
    protected void end() {
    	intake.setRollingSpeed(0);
		Logger.consoleLog("Speed", speed);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}

