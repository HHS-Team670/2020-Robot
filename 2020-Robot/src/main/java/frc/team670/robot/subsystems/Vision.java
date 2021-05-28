package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.RobotMap;

public class Vision extends VisionSubsystemBase{

    public Vision(){
        super(new double[] {0, 0, 240}, new double[] {179, 30, 255}, RobotMap.PCMODULE, RobotMap.VISION_LED_PCM, 
            new VisionShapePointList(
                new VisionShapePoint(-498.475, 431.0, 0, 1), 
                new VisionShapePoint(-250.825, 0.0, 0, 2), 
                new VisionShapePoint(250.825, 0.0, 0, 3), 
                new VisionShapePoint(498.475, 431, 0, 4)
                )
            );
    }
    
}
