package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.robot.commands.cameras.FlipDriverCameraMode;
import frc.team670.robot.commands.drive.teleop.FlipDriveAndCamera;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.Conveyor;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Indexer;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Turret;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.XboxButtons;

public class OI {

  private MustangController driverController;
  private Joystick operatorController;

  private JoystickButton toggleReverseDrive, toggleDriverCameraMode;

  private XKeys xkeys;

  public OI(DriveBase drivebase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Climber climber, Turret turret, MustangCoprocessor pi) {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveAndCamera());
    toggleDriverCameraMode = new JoystickButton(driverController, XboxButtons.B);
    toggleDriverCameraMode.whenPressed(new FlipDriverCameraMode());
    operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys(drivebase, intake, conveyor, indexer, shooter, climber, turret, pi);
  }

  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }

  /**
   * Sets the rumble on the driver controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time  The time to rumble for in seconds
   */
  public void rumbleDriverController(double power, double time) {
    rumbleController(driverController, power, time);
  }

  private void rumbleController(MustangController controller, double power, double time) {
    controller.rumble(power, time);
  }

  public MustangController getDriverController() {
    return driverController;
  }

  public Joystick getOperatorController() {
    return operatorController;
  }

}