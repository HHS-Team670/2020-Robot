import java.util.HashMap;

import org.usfirst.frc.team670.robot.Robot;
import org.usfirst.frc.team670.robot.commands.LoggingCommand;


/**
 *
 */
public class Deploy extends LoggingCommand {

	private boolean isDeploy;
	
	/*
	 * @param isDeploy true if it is the deploy, false if it is to pick up
	 */
    public Deploy(boolean isDeploy) {
    	this.isDeploy = isDeploy;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	logInitialize(new HashMap<String, Object>() {{
		}});
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.setDeploy(isDeploy);
    	logExecute(new HashMap<String, Object>() {{
		}});
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	logFinished(new HashMap<String, Object>() {{
		}});    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}

