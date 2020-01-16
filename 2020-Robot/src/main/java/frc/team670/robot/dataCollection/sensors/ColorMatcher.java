package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.team670.robot.constants.RobotConstants;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorMatcher {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
   * The device will be automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these are
   * here as a basic example.
   */
  public static final Color BLUE_TARGET = ColorMatch.makeColor(0.136, 0.412, 0.450);
  public static final Color YELLOW_TARGET = ColorMatch.makeColor(0.293, 0.561, 0.144);
  public static final Color RED_TARGET = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color GREEN_TARGET = ColorMatch.makeColor(0.196, 0.557, 0.246);  

  public static final int BLUE_COLOR_NUMBER = 0;
  public static final int YELLOW_COLOR_NUMBER = 1;
  public static final int RED_COLOR_NUMBER = 2;
  public static final int GREEN_COLOR_NUMBER = 3;


  private ColorMatchResult matchedResult = new ColorMatchResult(Color.kBlack, 0);

  public int colorNumber;

  // Rev Color threshold
  // blue 0.143, 0.427, 0.429
  // green 0.197, 0.561, 0.240
  // red 0.561, 0.232, 0.114
  // yellow 0.361, 0.524, 0.113

  public void init() {
    m_colorMatcher.addColorMatch(BLUE_TARGET);
    m_colorMatcher.addColorMatch(YELLOW_TARGET);
    m_colorMatcher.addColorMatch(RED_TARGET);
    m_colorMatcher.addColorMatch(GREEN_TARGET);   

    m_colorMatcher.setConfidenceThreshold(0.80);
  }

  public int detectColor() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and
     * can be useful if outputting the color to an RGB LED or similar. To read the
     * raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in well
     * lit conditions (the built in LED is a big help here!). The farther an object
     * is the more light from the surroundings will bleed into the measurements and
     * make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    matchedResult = match;

    if (match.color == BLUE_TARGET) {
      colorString = "Blue";
      colorNumber = BLUE_COLOR_NUMBER;
    } else if (match.color == YELLOW_TARGET) {
      colorString = "Yellow";
      colorNumber = YELLOW_COLOR_NUMBER;
    } else if (match.color == RED_TARGET) {
      colorString = "Red";
      colorNumber = RED_COLOR_NUMBER;
    } else if (match.color == GREEN_TARGET) {
      colorString = "Green";
      colorNumber = GREEN_COLOR_NUMBER;
    } else {
      colorString = "Unknown";
      colorNumber = -1;
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("Detected Color Number", colorNumber);

    return colorNumber;
  }
}