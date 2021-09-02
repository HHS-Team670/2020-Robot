package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.RobotMap;

public class Vision extends VisionSubsystemBase{

   // hexagon
    // new VisionShapePointList(
    //     new VisionShapePoint(-498.475, 431.0, 0, 1), 
    //     new VisionShapePoint(-250.825, 0.0, 0, 2), 
    //     new VisionShapePoint(250.825, 0.0, 0, 3), 
    //     new VisionShapePoint(498.475, 431, 0, 4))

    // horizontal rectangle
    // new VisionShapePointList(
    //     new VisionShapePoint(-255, 50, 0, 1),
    //     new VisionShapePoint(255, 0, 0, 2),
    //     new VisionShapePoint(-255, 0, 0, 3),
    //     new VisionShapePoint(255, 50, 0, 4))

    // vertical rectangle
    // new VisionShapePointList(
    //     new VisionShapePoint(-25, 510, 0, 1),
    //     new VisionShapePoint(-25, 0, 0, 2),
    //     new VisionShapePoint(25, 0, 0, 3),
    //     new VisionShapePoint(25, 510, 0, 4))

    public Vision(){
        super(new double[] {70, 235, 255}, new double[] {95, 255, 255}, RobotMap.PCMODULE, RobotMap.VISION_LED_PCM, new VisionShapePointList()
        // new VisionShapePointList(
        //     new VisionShapePoint(-255, 50, 0, 1),
        //     new VisionShapePoint(255, 0, 0, 2),
        //     new VisionShapePoint(-255, 0, 0, 3),
        //     new VisionShapePoint(255, 50, 0, 4))
        );
    }
    
}
