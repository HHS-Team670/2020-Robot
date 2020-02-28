package frc.team670.robot.commands.turret;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.subsystems.Turret;

/**
 * Rotate the turret to the forward limit in order to zero the encoder. 
 */
public class RotateToHome extends CommandBase implements MustangCommand {

    private Turret turret;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public RotateToHome(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(turret, HealthState.GREEN);
    }

    @Override
    public void execute(){
        Logger.consoleLog("Turret moving to zero");
        turret.moveByPercentOutput(0.05); // move very slowly until we hit the limit
    }

    @Override
    public boolean isFinished() {
        return turret.isForwardLimitSwitchTripped();
    }

    @Override
    public void end(boolean interrupted){
        Logger.consoleLog("Turret reached limit");
        turret.stop();
        if (!interrupted){
            turret.resetRotatorEncoderFromLimitSwitch();
            Logger.consoleLog("Turret encoder position is %s", turret.getRotatorEncoder().getPosition());
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}