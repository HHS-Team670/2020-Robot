package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.Indexer;

public class RunConveyor extends CommandBase implements MustangCommand {

    private Conveyor conveyor;
    private Indexer indexer;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean reversed;

    private long microSecondsSinceZeroBalls = -1;

    public RunConveyor(boolean reversed, Conveyor conveyor, Indexer indexer) {
        this.conveyor = conveyor;
        this.indexer = indexer;
        this.reversed = reversed;
        addRequirements(conveyor);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(conveyor, HealthState.GREEN);
    }

    @Override
    public void execute() {
		conveyor.run(reversed);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.consoleLog("Indexer system emptied");
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        if(indexer.getTotalNumBalls() == 0 && microSecondsSinceZeroBalls == -1){
            microSecondsSinceZeroBalls =  RobotController.getFPGATime();
        }
        if(microSecondsSinceZeroBalls != -1 && RobotController.getFPGATime() - microSecondsSinceZeroBalls >= 500000){
            microSecondsSinceZeroBalls = -1;
            return true;
        }
        return false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}