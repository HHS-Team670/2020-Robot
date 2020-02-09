package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

/**
 *
 */
public class RunIntake extends InstantCommand implements MustangCommand {

	Map<MustangSubsystemBase, HealthState> healthReqs;
	private double speed;
	private Intake intake;

	public RunIntake (double speed, Intake intake) {
		this.speed = speed;
		this.intake = intake;
		healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.YELLOW);
		addRequirements(intake);
	}
	
	public void initialize() {
		intake.roll(speed);
	}


	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		return healthReqs;
	}
}

