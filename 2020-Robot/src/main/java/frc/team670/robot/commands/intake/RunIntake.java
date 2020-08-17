package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

/**
 * Runs the intake of the robot
 */
public class RunIntake extends CommandBase implements MustangCommand {

	Map<MustangSubsystemBase, HealthState> healthReqs;
	private boolean reversed;
	private Intake intake;
	private int countWasJammed;

	/**
	 * Initializes this command from the parameters
	 * @param reversed true to run the intake in reverse (out), false to run it
	 *                 normally (in)
	 * @param intake the intake of the robot
	 */
	public RunIntake(boolean reversed, Intake intake) {
		this.reversed = reversed;
		this.intake = intake;
		healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.YELLOW);
		addRequirements(intake);
		countWasJammed = 0;
	}

	public void initialize() {
		intake.roll(reversed);
	}

	public void execute() {
		SmartDashboard.putNumber("count jammed", countWasJammed);
		if (intake.isJammed()) {
			countWasJammed = 150;
		}
		if (countWasJammed > 0) {
			intake.roll(!reversed);
			countWasJammed--;
		} else {
			intake.roll(reversed);
		}
	}

	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		return healthReqs;
	}
}
