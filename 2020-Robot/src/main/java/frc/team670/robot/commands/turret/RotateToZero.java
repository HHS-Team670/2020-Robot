package frc.team670.robot.commands.turret;
import frc.team670.robot.subsystems.Turret;

/**
 * RotateTurret, except with zero passed into it
 * @see RotateTurret
 */
public class RotateToZero extends RotateTurret {

    public RotateToZero(Turret t) {
        super(0, t, false);
    }
}