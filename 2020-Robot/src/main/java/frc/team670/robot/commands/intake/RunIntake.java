package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
/**
 *
 */
public class RunIntake extends CommandBase implements MustangCommand {

	Map<MustangSubsystemBase, HealthState> healthReqs;
	private boolean reversed;
	private Intake intake;

	/**
	 * @param reversed true to run the intake in reverse, false to run it normally
	 */
	public RunIntake (boolean reversed, Intake intake) {
		this.reversed = reversed;
		this.intake = intake;
		healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.YELLOW);
		addRequirements(intake);
	}
	
	public void initialize() {
		intake.roll(reversed);
	}


	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		return healthReqs;
	}
}

