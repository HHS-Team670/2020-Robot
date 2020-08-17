package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;

/**
 * Deploys the intake
 */
public class DeployIntake extends InstantCommand implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean isDeploy;
    private Intake intake;

    /**
     * @param isDeploy true if it is to deploy, false if it is to pick up
     * @param intake the intake of the robot
     */
    public DeployIntake(boolean isDeploy, Intake intake) {
        this.isDeploy = isDeploy;
        this.intake = intake;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        addRequirements(intake);
    }

    public void initialize() {

    }

    public void execute(){
        intake.deploy(isDeploy);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
