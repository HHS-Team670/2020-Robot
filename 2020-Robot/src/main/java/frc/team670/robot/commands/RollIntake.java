
import java.util.HashMap;

import org.usfirst.frc.team670.robot.Robot;
import org.usfirst.frc.team670.robot.commands.LoggingCommand;


/**
 *
 */
public class RollIntake extends LoggingCommand {

	private double speed, seconds;
	
    public RollIntake(double speed, double seconds) {
    	this.speed = speed;
    	this.seconds = seconds;
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        setTimeout(seconds);
		logInitialize(new HashMap<String, Object>() {{
			put("Speed", speed);
			put("Seconds", seconds);
		}});
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.rollIntake(speed);
		logExecute(new HashMap<String, Object>() {{
			put("Speed", speed);
			put("Seconds", seconds);
		}});
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.driveIntake(0);
    	logFinished(new HashMap<String, Object>() {{
			put("Speed", speed);
			put("Seconds", seconds);
		}});
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.intake.driveIntake(0);
    }
}

