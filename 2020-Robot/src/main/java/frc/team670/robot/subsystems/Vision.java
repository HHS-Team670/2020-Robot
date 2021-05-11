package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.RobotMap;

public class Vision extends VisionSubsystemBase{

    public Vision(){
        super(new double[] {}, new double[] {}, new double[] {}, RobotMap.PCMODULE, RobotMap.VISION_LED_PCM);
    }
    
}
