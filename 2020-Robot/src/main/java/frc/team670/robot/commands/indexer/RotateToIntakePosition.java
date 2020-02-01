package frc.team670.robot.commands.indexer;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Run this command before you run IntakeOneBall
 */
public class RotateToIntakePosition extends CommandBase implements MustangCommand{

    private double speed;
    private Indexer indexer;
    private int goal;
    //Indexer's position when it first sees the right chamber
    private double firstReachedPos;
    //When the chamber first shows that it's the correct chamber (not right positoin yet though)
    private boolean reached;
    //For isFinished()
    private boolean done;

    public RotateToIntakePosition(){
        speed = 0.8;
        addRequirements(RobotContainer.indexer);
        indexer = RobotContainer.indexer;
        goal = indexer.getIntakeChamber();
        firstReachedPos = 0;
        reached = false;
        done = false;
    }

    @Override
    public void execute() {
        indexer.setSpeed(speed);
        if (indexer.getTopChamber() == goal) {
            if (reached) {
                if (indexer.getPosition() > firstReachedPos + 0.1) {
                    done = true;
                }
            }
            firstReachedPos = indexer.getPosition();
            reached = true;
        }
    }

    @Override
    public void end(boolean isInteruppted) {
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}