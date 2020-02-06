package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

/**
 *
 */
public class RollIntakeTest extends WaitCommand implements MustangCommand {

	private double speed;
	private Intake intake;

	public RollIntakeTest (double speed, Intake intake) {
		//For testing purposes, just runs intake for 3 seconds, use sensor for isFinished later
		super(3);
		this.speed = speed;
		this.intake = intake;
		addRequirements(intake);
    }


	public void execute() {
		intake.roll(speed);
	}

	protected void end() {
		intake.roll(0);
	}

	protected void interrupted() {
		end();
	}


	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		Map<MustangSubsystemBase, HealthState> healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.GREEN);
		return healthReqs;
	}
}

