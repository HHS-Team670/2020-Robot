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
 * Rotate the turret to the reverse limit in order to zero the encoder. 
 */
public class RotateToHome extends CommandBase implements MustangCommand {

    private Turret turret;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    boolean hasReachedLimit = false;

    public RotateToHome(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(turret, HealthState.GREEN);
    }

    @Override
    public void execute(){
        Logger.consoleLog("Turret moving to zero");
        turret.moveByPercentOutput(0.15); // move very slowly until we hit the limit
        if(turret.isForwardLimitSwitchTripped()){
            turret.stop();
            turret.resetRotatorEncoderFromLimitSwitch();
            hasReachedLimit = true;
        }
        // if(hasReachedLimit){
        //     // turret.moveByPercentOutput(-0.05);
        //     turret.setSystemTargetAngleInDegrees(-20);
        // }
    }

    @Override
    public boolean isFinished() {
        // Logger.consoleLog("turretAngle %s %s", turret.hasReachedTargetPosition(), hasReachedLimit);
        return hasReachedLimit;
    }

    @Override
    public void end(boolean interrupted){
        Logger.consoleLog("Turret reached limit");
        turret.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}