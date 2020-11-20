package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.robot.commands.cameras.FlipDriverCameraMode;
import frc.team670.robot.commands.drive.teleop.FlipDriveAndCamera;
import frc.team670.robot.commands.drive.teleop.FlipDriveDirection;
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

/**
This class is the glue that binds the controls on the physical operator interface to the commands and command groups that allow control of the robot.
*/
public class OI {

  private MustangController driverController;
  private Joystick operatorController;

  private JoystickButton toggleReverseDrive;

  private XKeys xkeys;

  /**
   * Used to create the joysticks used to control this robot
   * @param drivebase the drivebase to control
   * @param intake the intake to control
   * @param conveyor the conveyor to run
   * @param indexer the indexer to run/control
   * @param shooter the shooter to run/control
   * @param climber the climber to run
   * @param turret the turret to run/control
   * @param pi the coprocessor, specificallly pi
   */
  public OI(DriveBase drivebase, Intake intake, Conveyor conveyor, Indexer indexer, Shooter shooter, Climber climber, Turret turret, MustangCoprocessor pi) {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveDirection());
    operatorController = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys(drivebase, intake, conveyor, indexer, shooter, climber, turret, pi);
  }

  /**
   * Used to check if the quickturn button on the driver controller is pressed
   * @return true if the button is pressed
   */
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

    /**
   * Notifies the driver controller by rumbling it
   * 
   */
  public void notifyDriverController(double power, double time) {
    rumbleDriverController(power, time);
    rumbleDriverController(0, 1);
    rumbleDriverController(power, time);
  }

  private void rumbleController(MustangController controller, double power, double time) {
    controller.rumble(power, time);
  }

  /**
   * Used to get the driver controller
   */
  public MustangController getDriverController() {
    return driverController;
  }

  /**
   * Used to get the operator controller
   * @return
   */
  public Joystick getOperatorController() {
    return operatorController;
  }

}