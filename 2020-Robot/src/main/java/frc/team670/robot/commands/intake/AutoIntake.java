package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;

/**
 *
 */
public class AutoIntake extends CommandBase implements MustangCommand {

	Map<MustangSubsystemBase, HealthState> healthReqs;
	private boolean reversed;
	private Intake intake;
	private Indexer indexer;
    private Conveyor conveyor;
	private int countWasJammed;

	/**
	 * @param reversed true to run the intake in reverse (out), false to run it
	 *                 normally (in)
	 */
	public AutoIntake(Intake intake, Conveyor conveyor, Indexer indexer) {
		this.intake = intake;
        this.conveyor = conveyor;
		this.indexer = indexer;
		healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.YELLOW);
        healthReqs.put(conveyor, HealthState.YELLOW);
		addRequirements(intake, conveyor);
		countWasJammed = 0;
	}

	public void initialize() {
		intake.roll(reversed);
        conveyor.run(false);
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
		// if(conveyor.isBallInConveyor()){
		// 	conveyor.stop();
		// }
	}

	@Override
	public boolean isFinished(){
		return indexer.ballInChamber(2);
	}

	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		return healthReqs;
	}
}
