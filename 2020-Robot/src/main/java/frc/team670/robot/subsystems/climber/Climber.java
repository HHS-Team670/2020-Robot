package frc.team670.robot.subsystems.climber;

import frc.team670.robot.subsystems.MustangSubsystemBase;

/**
 * @author Pallavi & Eugenia
 */

public class Climber extends MustangSubsystemBase {

    private Pull rightPull;
    private Pull leftPull;
    private boolean leftPullHookedOnBar;
    private boolean rightPullHookedOnBar;

    public Climber() {
        // This has the left pull set to be inverted, and right pull is not inverted,
        // change if needed
        this(new Pull(true), new Pull(false));
    }

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

    public boolean hookOnBar() {
        leftPullHookedOnBar = leftPull.hookOnBar();
        rightPullHookedOnBar = rightPull.hookOnBar();
        return rightPullHookedOnBar && leftPullHookedOnBar;
    }

    public boolean isAtTarget() {
        return leftPull.isAtTarget() && rightPull.isAtTarget();
    }

    @Override
    public HealthState checkHealth() {
        HealthState left = leftPull.getHealth(false), right = rightPull.getHealth(false);
        if (left.equals(HealthState.UNKNOWN)) {
            left = leftPull.getHealth(true);
        }
        if (right.equals(HealthState.UNKNOWN)) {
            right = rightPull.getHealth(true);
        }
        if (left != HealthState.GREEN || right != HealthState.GREEN) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

}