package frc.team670.robot.commands.turret;

import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.RobotContainer;

public class ManualControlAndAutoRotate extends AutoRotate{

    public Turret turret;

    public ManualControlAndAutoRotate(Turret turret, Vision pi, DriveBase driveBase){
        super(turret, pi, driveBase);
        this.turret = turret;
    }

    @Override
    public void execute(){
        if (RobotContainer.getOperatorController().getRawButton(4)) {
            double power = RobotContainer.getOperatorController().getZ()/2;
            if(Math.abs(power) > 0.025){
              turret.moveByPercentOutput(power);
            }
            else{
              turret.moveByPercentOutput(0);
            }
          }
          else  if (RobotContainer.getDriverController().getRawButton(10)) {
            double power = RobotContainer.getDriverController().getRightStickY()/2;
            if(Math.abs(power) > 0.025){
              turret.moveByPercentOutput(power);
            }
            else{
              turret.moveByPercentOutput(0);
            }
          }
          else{
            executeAutoRotate();
          }

    }
    
}
