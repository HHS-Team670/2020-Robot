package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Stops the intake
 */
public class StopIntake extends InstantCommand implements MustangCommand {

	Map<MustangSubsystemBase, HealthState> healthReqs;
	private Intake intake;

	/**
	 * Initializes this command from the given intake
	 * @param intake the intake of the robot
	 */
	public StopIntake (Intake intake) {
		this.intake = intake;
		healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.YELLOW);
		addRequirements(intake);
	}
	
	public void initialize() {
		intake.stop();
	}


	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		return healthReqs;
	}
}

