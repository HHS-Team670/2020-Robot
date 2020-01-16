package frc.team670.robot.constants;

import frc.team670.robot.utils.MustangController;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.robot.commands.cameras.FlipDriverCameraMode;
import frc.team670.robot.commands.drive.teleop.FlipDriveAndCamera;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.MustangController.XboxButtons;

public class OI {

    private MustangController driverController;

    private JoystickButton toggleReverseDrive, toggleDriverCameraMode;
    private JoystickButton armToNeutral;  

    public OI() {
        driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
        // operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
        toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
        toggleReverseDrive.whenPressed(new FlipDriveAndCamera());
        toggleDriverCameraMode = new JoystickButton(driverController, XboxButtons.B);
        toggleDriverCameraMode.whenPressed(new FlipDriverCameraMode());
      
      }

      public boolean isQuickTurnPressed() {
        return driverController.getRightBumper();
      }

      public boolean isAPressed() {
        return driverController.getAButton();
      }


      /**
   * Sets the rumble on the driver controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleDriverController(double power, double time) {
    rumbleController(driverController, power, time);
  }

  /**
   * Sets the rumble on the operator controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleOperatorController(double power, double time) {
    // rumbleController(operatorController, power, time);
  }

  private void rumbleController(MustangController controller, double power, double time) {
    controller.rumble(power, time);
  }

  public MustangController getDriverController(){
      return driverController;
  }

}