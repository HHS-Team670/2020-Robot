package frc.team670.robot.subsystems.climber;

import frc.team670.robot.subsystems.climber.Pull;
import frc.team670.robot.subsystems.MustangSubsystemBase;

/**
 * @author Pallavi, Eugenia, & Sofia
 */

public class Climber extends MustangSubsystemBase {

    private Pull rightPull;
    private Pull leftPull;
    private boolean leftPullHookedOnBar;
    private boolean rightPullHookedOnBar;
    private boolean isExtending; // true: extending, false: retracting

    public Climber(Pull rightPull, Pull leftPull) {
        this.leftPull = leftPull;
        this.rightPull = rightPull;
        leftPullHookedOnBar = false;
        rightPullHookedOnBar = false;
    }

    public void set(double speed) {
        leftPull.setPower(speed);
        rightPull.setPower(speed);
    }

    public Pull getRightPull() {
        return rightPull;
    }

    public Pull getLeftPull() {
        return leftPull;
    }

    public void climb(double heightCM) {
        leftPull.climb(heightCM);
        rightPull.climb(heightCM);

    }

    public void setIsExtending(boolean isExtending) {
        this.isExtending = isExtending;
        if (isExtending) {
            leftPull.solenoidOn();
            rightPull.solenoidOn();
        } else {
            leftPull.solenoidOff();
            rightPull.solenoidOff();
        }

    }

    public boolean hookOnBar() {
        leftPullHookedOnBar = leftPull.isHookedOnBar();
        rightPullHookedOnBar = rightPull.isHookedOnBar();
        return rightPullHookedOnBar && leftPullHookedOnBar;
    }

    public boolean isAtTarget() {
        return leftPull.isAtTarget() && rightPull.isAtTarget();
    }

    @Override
    public HealthState checkHealth() {
        if (leftPull.getHealth(false) == HealthState.RED || rightPull.getHealth(false) == HealthState.RED) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

}