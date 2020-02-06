package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.ColorMatcher;
import frc.team670.robot.utils.motorcontroller.MotorConfig;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
/**
 * Represents the subsystem for spinning the control panel/color wheel.
 * 
 * @author Antonio Cuan, Katelyn Yap, ctychen
 */
public class ColorWheelSpinner extends MustangSubsystemBase {

    private static ColorMatcher colorMatch;
    private SparkMAXLite rotator;

    public final double MOTOR_SPEED = 0.1; // TODO: tune for actual motor

    public ColorWheelSpinner() {
        colorMatch = new ColorMatcher();
        rotator = new SparkMAXLite(RobotMap.COLOR_WHEEL_MOTOR_ID, MotorConfig.Motor_Type.NEO_550);
    }

    public void setSpeed(double motorSpeed) {
        rotator.set(motorSpeed);
    }

    public int detectColor() {
        return colorMatch.detectColor();
    }

    /**
     * @return GREEN if everything is fine. Otherwise, YELLOW if there is a problem with the color sensor, but nothing else. RED if there are issues with the motor or controller.
     */
    @Override
    public HealthState checkHealth() {
        if (rotator.getLastError()!=CANError.kOk){
            DriverStation.reportWarning("RED error: Problem with color wheel spinner motor", false);
            return HealthState.RED;
        }
        if (colorMatch == null){
            DriverStation.reportWarning("YELLOW Warning: Color sensor not found", false);
            return HealthState.YELLOW;
        }
        return HealthState.GREEN;
    }

    public void mustangPeriodic(){

    }
}