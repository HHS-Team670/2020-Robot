package frc.team670.robot.utils;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Represents an Xbox controller
 */
public class MustangController extends XboxController {

    private Notifier rumbler;
    private boolean isRumbling;
    private long targetRumbleTime;

    /**
     * The dpad states for xbox
     */
    public enum DPadState {
        NEUTRAl, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT;
    }

    /**
     * Different XBox buttons with their ids
     */
    public static class XboxButtons {
        // Controller Buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RIGHT_BUMPER = 6;
        public static final int LEFT_BUMPER = 5;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int LEFT_JOYSTICK_BUTTON = 9;
        public static final int RIGHT_JOYSTICK_BUTTON = 10;

        // Controller Axes
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int LEFT_STICK_X = 0;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int LEFT_STICK_Y = 1;
        /**
         * Left = Positive, Right = Negative [-1, 1]
         */
        public static final int LEFT_TRIGGER_AXIS = 2;
        /**
         * Pressed = Positive [0, 1]
         */
        public static final int RIGHT_TRIGGER_AXIS = 3;
        /**
         * Left = Negative, Right = Positive [-1, 1]
         */
        public static final int RIGHT_STICK_X = 4;
        /**
         * Up = Negative, Down = Positive [-1, 1]
         */
        public static final int RIGHT_STICK_Y = 5;
    }

    /**
     * Used to create a mustang controller (xbox) on a spcific port
     * @param port the port to which the xbox is connected
     */
    public MustangController(int port) {
        super(port);
        isRumbling = false;
        targetRumbleTime = System.currentTimeMillis() - 10;
        rumbler = new Notifier(new Runnable() {
            public void run() {
                if(isRumbling) {
                    checkRumble();
                }
            }
          });
        rumbler.startPeriodic(0.125);
    }

    /**
     * Used to get left stick x
     */
    public double getLeftStickX() {
        return super.getRawAxis(XboxButtons.LEFT_STICK_X);
    }

    /**
     * Used to get left joystick Y
     * @return
     */
    public double getLeftStickY() {
        return super.getRawAxis(XboxButtons.LEFT_STICK_Y);
    }

    /**
     * Used to get the left trigger axis
     * @return
     */
    public double getLeftTriggerAxis() {
        return super.getTriggerAxis(Hand.kLeft);
    }

    /**
     * Used to get the right trigger axis
     */
    public double getRightTriggerAxis() {
        return super.getTriggerAxis(Hand.kRight);
    }

    /**
     * Used to get right stick X
     * @return
     */
    public double getRightStickX() {
        return super.getRawAxis(XboxButtons.RIGHT_STICK_X);
    }

    /**
     * Used to get the right stick Y
     * @return
     */
    public double getRightStickY() {
        return super.getRawAxis(XboxButtons.RIGHT_STICK_Y);
    }

    /**
     * Used to check if the left bumper is pressed
     * @return
     */
    public boolean getLeftBumper() {
        return super.getBumper(Hand.kLeft);
    }

    /**
     * Used to check if the right bumper is pressed
     * @return
     */
    public boolean getRightBumper() {
        return super.getBumper(Hand.kRight);
    }

    /**
     * Used to check if the left joystick button is pressed
     * @return
     */
    public boolean getLeftJoystickButton() {
        return super.getStickButton(Hand.kLeft);
    }

    /**
     * Used to check if the right joystick button is pressed
     * @return
     */
    public boolean getRightJoystickButton() {
        return super.getStickButton(Hand.kRight);
    }

    /**
     * Used to get the POV
     * @return
     */
    public int getPOVValue() {
        return super.getPOV();
    }

    /**
     * Sets the rumble on the controller
     * 
     * @param power The desired power of the rumble [0, 1]
     * @param time The time to rumble for in seconds
     */
    public void rumble(double power, double time) {
        setRumblePower(power);
        isRumbling = true;
        targetRumbleTime = System.currentTimeMillis() + (long)(time * 1000);
    }

    /**
     * Sets the rumble on the controller
     * 
     * @param power The desired power of the rumble [0, 1]
     * @param time  The time to rumble for in seconds
     */
    private void setRumblePower(double power) {
        setRumble(RumbleType.kLeftRumble, power);
        setRumble(RumbleType.kRightRumble, power);
    }

    private void checkRumble() {
        if(System.currentTimeMillis() >= targetRumbleTime) {
            setRumblePower(0);
            isRumbling = false;
        }
    }

    // gets angle of the DPad on the XBox controller pressed with increments of 45
    // degree angle.
    // returns neutal or -1 when nothing is pressed
    public DPadState getDPadState() {

        int angle = super.getPOV();

        if (angle == 0) {
            return DPadState.UP;
        } else if (angle == 45) {
            return DPadState.UP_RIGHT;
        } else if (angle == 90) {
            return DPadState.RIGHT;
        } else if (angle == 135) {
            return DPadState.DOWN_RIGHT;
        } else if (angle == 180) {
            return DPadState.DOWN;
        } else if (angle == 225) {
            return DPadState.DOWN_LEFT;
        } else if (angle == 270) {
            return DPadState.LEFT;
        } else if (angle == 315) {
            return DPadState.UP_LEFT;
        } else {
            return DPadState.NEUTRAl;
        }

    }

}