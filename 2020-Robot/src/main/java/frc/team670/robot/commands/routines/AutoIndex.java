package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
public class AutoIndex extends CommandBase implements MustangCommand {

	Map<MustangSubsystemBase, HealthState> healthReqs;
	private boolean reversed;
	private Intake intake;
	private Indexer indexer;
    private Conveyor conveyor;
	private int countWasJammed;
	private int targetBalls; 
	private DriverStation ds;

	public AutoIndex(Intake intake, Conveyor conveyor, Indexer indexer, int targetBalls) {
		this.intake = intake;
		this.targetBalls = targetBalls;
        this.conveyor = conveyor;
		this.indexer = indexer;
		healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
		healthReqs.put(intake, HealthState.YELLOW);
        healthReqs.put(conveyor, HealthState.YELLOW);
		addRequirements(intake, conveyor);
		countWasJammed = 0;
		ds = DriverStation.getInstance();
	}

	public AutoIndex(Intake intake, Conveyor conveyor, Indexer indexer) {
		this(intake, conveyor, indexer, 4);
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
	}

	@Override
	public boolean isFinished(){
		return indexer.getTotalNumBalls() == targetBalls;
	}

	@Override
	public void end(boolean interrupted){
		intake.stop();
		intake.deploy(false);
		conveyor.stop();
	} 

	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		return healthReqs;
	}
}
