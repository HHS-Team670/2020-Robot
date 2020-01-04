package frc.team670.robot.commands.cameras;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.RobotContainer;

/**
 *  Flips the camera: The front camera or the camera on the back
 */
public class FlipCamera extends InstantCommand{

    /**
     * Driver camera direction (true = front, false = back). Starts false because robot starts backwards.
     */
    private static boolean cameraDirection = false;
    private static int camNumber = 0;

    private FlipCamera(){
        super();
    }

    // called once when the command executes
    public void initialize(){
        flipCameraDirection();
        RobotContainer.oi.rumbleDriverController(0.7, 0.2);
    }

    public static void flipCameraDirection() {
        cameraDirection = !cameraDirection;
        camNumber = (camNumber + 1) % 2;
        SmartDashboard.putString("camera-source", camNumber+"");
    }

    /**
     * Driver camera direction (true = front, false = back). Starts false because robot starts backwards.
     */
    public static boolean getCameraDirection() {
        return cameraDirection;
    }
}