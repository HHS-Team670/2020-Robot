package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.RobotMap;

public class Vision extends VisionSubsystemBase{

    public Vision(){
        super(new double[] {0, 0, 240}, new double[] {179, 30, 255}, new double[] {}, RobotMap.PCMODULE, RobotMap.VISION_LED_PCM);
    }
    
}
