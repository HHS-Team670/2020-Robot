package frc.team670.robot.utils.math;

/**
 * Another way to supply power values to subsystems
 */
public class DrivePower {
	private double left;
	private double right;

	/**
	 * Used to create a drivepower based on left power and right power
	 * @param left the left power 
	 * @param right the right power
	 */
	public DrivePower(double left, double right) {
		this.left = left;
		this.right = right;
	}

	/**
	 * Gets the left power
	 * @return double leftPower
	 */
	public double getLeft() {
		return left;
	}

	/**
	 * Used to get the right power
	 * @return double rightPower
	 */
	public double getRight() {
		return right;
	}

}
