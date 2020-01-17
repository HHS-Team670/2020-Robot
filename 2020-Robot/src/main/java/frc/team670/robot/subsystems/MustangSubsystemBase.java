package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.team670.robot.commands.MustangCommandBase;
import frc.team670.robot.RobotContainer;


public abstract class MustangSubsystemBase extends SubsystemBase{

    private HealthState lastHealthState;

    public MustangSubsystemBase(){
        RobotContainer.addSubsystem(this);
    }

    
    /**
     * Represents possible conditions a MustangSubsystem can be in.
     * Each MustangSubsystem should define what the States mean for it specifically.
     */
    public enum HealthState{
        GREEN(0), YELLOW(1), RED(2);

        private final int ID;

        HealthState(int id) {
          ID = id;
        }
    
        /**
         * Gets the ID of the state. 
         */
        public int getId() {
          return ID;
        }

    }

    public HealthState getHealth(){
        return this.lastHealthState;
    }

    protected abstract HealthState checkHealth();

    public void setHealthState(){
        lastHealthState = checkHealth();
    }

    public void initDefaultCommand(MustangCommandBase command){
        CommandScheduler.getInstance().setDefaultCommand(this, command);
    }
  

}