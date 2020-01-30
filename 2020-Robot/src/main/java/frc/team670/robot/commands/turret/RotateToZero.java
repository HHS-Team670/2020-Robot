package frc.team670.robot.commands.turret;
import frc.team670.robot.subsystems.Turret;

public class RotateToZero extends RotateTurret {
    public RotateToZero(Turret t) {
        super(t.getZeroPoint(), t);
    }
}