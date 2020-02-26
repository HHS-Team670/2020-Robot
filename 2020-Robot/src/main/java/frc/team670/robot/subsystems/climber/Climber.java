package frc.team670.robot.subsystems.climber;

import frc.team670.robot.subsystems.climber.Pull;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team670.robot.subsystems.MustangSubsystemBase;

/**
 * @author Pallavi, Eugenia, & Sofia
 */

public class Climber extends MustangSubsystemBase {

    private Pull pull;
    private boolean pullIsHookedOnBar;
    private boolean isExtending; // true: extending, false: retracting

    public Climber(Solenoid deployer) {
        this.pull = new Pull(deployer);
        pullIsHookedOnBar = false;
    }

    public void set(double speed) {
        pull.setPower(speed);
    }

    public Pull getPull() {
        return pull;
    }

    /**
     * 
     * @param heightCM height to climb to, in centimeters
     */
    public void climb(double heightCM) {
        pull.climb(heightCM);
    }

    /**
     * 
     * @param isExtending true to deploy the pull (turn solenoid on)
     */
    public void setExtending(boolean isExtending) {
        this.isExtending = isExtending;
        if (isExtending) {
            pull.solenoidOn();
        } else {
            pull.solenoidOff();
        }
    }

    public boolean hookOnBar() {
        return pull.isHookedOnBar();
    }

    public boolean isAtTarget() {
        return pull.isAtTarget();
    }

    @Override
    public HealthState checkHealth() {
        if (pull.getHealth(false) == HealthState.RED) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

}