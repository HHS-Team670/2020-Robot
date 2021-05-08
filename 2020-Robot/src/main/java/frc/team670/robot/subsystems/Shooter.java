/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.math.interpolable.InterpolatingDouble;
import frc.team670.robot.utils.math.interpolable.InterpolatingTreeMap;
import frc.team670.robot.utils.math.interpolable.LinearRegression;
import frc.team670.robot.utils.math.interpolable.PolynomialRegression;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

/**
 * Represents a 2-stage shooter, with 1st stage using a VictorSPX and 2-NEO 2nd
 * stage with SparkMax controllers.
 * 
 * @author ctychen, Pallavi Das
 */
public class Shooter extends MustangSubsystemBase {

  private SparkMAXLite mainController, followerController;
  private List<SparkMAXLite> controllers;

  private CANEncoder stage2_mainEncoder;
  private CANPIDController stage2_mainPIDController;

  private double targetRPM = 2500; // Will change later if we adjust by distance
  private static double DEFAULT_SPEED = 2500;

  private static double MIN_RPM = 2000;
  private static double MAX_RPM = 3100;

  private double speedAdjust = 0; // By default, we don't adjust, but this may get set later

  private static double MAX_SHOT_DISTANCE_METERS = 8.6868; // = 28-29ish feet

  private static final double PULLEY_RATIO = 2; // Need to check this

  private boolean ballHasBeenShot;
  private int shootingCurrentCount = 0;

  private static final double NORMAL_CURRENT = 0; // TODO: unknown

  // Stage 2 values, as of 2/17 testing
  private static final double V_P = 0.000100;
  private static final double V_I = 0.0;
  private static final double V_D = 0.0;
  private static final double V_FF = 0.000183;
  private static final double RAMP_RATE = 1.0;

  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> FLYWHEEL_RPM_MAP = new InterpolatingTreeMap<>();

  // The following is for using fancy interpolation based on 254's 2017 code, where we sample 
  // a ton and build a polynomial regression.

  // Format: {Distance from target in meters, RPM}
  // Distance currently from bumper
  private static final double[][] FLYWHEEL_RPM_AT_DISTANCE = { 

    // { 3.32232, 2125},  // 10.9 ft  2125 rpm 
    // { 4.572,  2275 }, // 15 ft  2275 rpm 
    // { 7.3152, 2575 }, // 24 ft 2575 rpm 
    // { 8.6868, 2725 } // 28.5 ft 2725 rpm 

    // { 3.32232, 2250},  // 10.9 ft  2125 rpm 
    // { 4.572,  2350 }, // 15 ft  2275 rpm 
    // { 7.3152, 2650 }, // 24 ft 2575 rpm 
    //  {9.4488, 3100} //edward list

    { 3.32232, 2125},  // 10.9 ft  2125 rpm 
    { 4.572,  2275 }, // 15 ft  2275 rpm 
    { 7.3152, 2575 }, // 24 ft 2575 rpm 
    { 8.6868, 2725 } // 28.5 ft 2725 rpm  2550

  };

  static {
    for (double[] pair : FLYWHEEL_RPM_AT_DISTANCE) {
      FLYWHEEL_RPM_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
    DEFAULT_SPEED = FLYWHEEL_RPM_MAP.getInterpolated(new InterpolatingDouble(MAX_SHOT_DISTANCE_METERS)).value;
  }

  // For now this should be enough: a linear regression for the relationship between distance
  // and RPM from this data should have a correlation coefficient of 0.9999 so it should be fine

  private static final double[] measuredDistancesMeters = {
    3.32232,  // 10.9 ft  2125 rpm 
    4.572, // 15 ft  2275 rpm 
    7.3152, // 24 ft 2575 rpm
    8.6868, 
    9.4488,// trench (28-29ft)
  };

  private static final double[] measuredRPMs = {
    2125,  // 10.9 ft  2125 rpm 
    2275, // 15 ft  2275 rpm 
    2575, // 24 ft 2575 rpm 
    2725, 
    3100
  };

  private static final PolynomialRegression speedAtDistance = new PolynomialRegression(measuredDistancesMeters, measuredRPMs, 4);

  private static final int VELOCITY_SLOT = 0;

  /**
   * constructor
   */
  public Shooter() {

    SmartDashboard.putNumber("Stage 2 Velocity Setpoint", 0.0);
    SmartDashboard.putNumber("Stage 2 FF", 0.0);
    SmartDashboard.putNumber("Stage 2 P", 0.0);
    SmartDashboard.putNumber("Stage 2 Ramp Rate", 0.0);
    SmartDashboard.putNumber("Stage 2 speed", 0.0);

    controllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SHOOTER_MAIN,
        RobotMap.SHOOTER_FOLLOWER, true, Motor_Type.NEO);

    mainController = controllers.get(0);
    followerController = controllers.get(1);

    stage2_mainEncoder = mainController.getEncoder();
    stage2_mainPIDController = mainController.getPIDController();

    stage2_mainPIDController.setP(V_P, VELOCITY_SLOT);
    stage2_mainPIDController.setI(V_I, VELOCITY_SLOT);
    stage2_mainPIDController.setD(V_D, VELOCITY_SLOT);
    stage2_mainPIDController.setFF(V_FF, VELOCITY_SLOT);
  }

  /**
   * gets stage 2 velocity
   * @return
   */
  public double getStage2Velocity() {
    return stage2_mainEncoder.getVelocity();
  }

  /**
   * runs the shooter
   */
  public void run() {
    SmartDashboard.putNumber("Stage 2 speed", mainController.getEncoder().getVelocity());
    stage2_mainPIDController.setReference(targetRPM + speedAdjust, ControlType.kVelocity);
  }

  /**
   * @param setRamp true if we want a ramp rate (use this for getting the shooter
   *                up to speed), false when we're ready to shoot and don't need
   *                one
   */
  public void setRampRate(boolean setRamp) {
    if (setRamp) {
      mainController.setClosedLoopRampRate(RAMP_RATE);
    } else {
      mainController.setClosedLoopRampRate(0);
    }
  }

  /**
   * @param targetRRMP sets velocity target for shoorer
   */
  public void setVelocityTarget(double targetRPM) {
    this.targetRPM = targetRPM;
  }

  /**
   * 
   * @return default RPM speed
   */
  public double getDefaultRPM(){
    return this.DEFAULT_SPEED;
  }

  /**
   * 
   * @param diff The amount to change the current RPM adjust by, positive for increasing and negative to decrease
   */
  public void adjustRPMAdjuster(double diff) {
    if(((diff > 0 && speedAdjust < 400) || (diff < 0 && speedAdjust > -400))){
      this.speedAdjust += diff;
      if(stage2_mainEncoder.getVelocity() > 300){
        run();
      }

    }
  }

  /**
   * 
   * @param distance In meters, the distance we are shooting at
   * @return The predicted "best fit" RPM for the motors to spin at based on the distance,
   * calculated from the linear regression
   */
  public double getTargetRPMForDistance(double distance){
    double predictedVal = speedAtDistance.predict(distance);
    Logger.consoleLog("predictedVal: %s", predictedVal);
    double expectedSpeed = Math.max(Math.min(predictedVal, MAX_RPM), MIN_RPM);
    SmartDashboard.putNumber("expectedSpeed", expectedSpeed);
    return predictedVal;
  }

  /**
   * stops running shooter
   */
  public void stop() {
    stage2_mainPIDController.setReference(0, ControlType.kDutyCycle);
  }

  /**
   * 
   * @return if shooter is running at target speed
   */
  public boolean isUpToSpeed() {
    return MathUtils.doublesEqual(getStage2Velocity(), targetRPM + this.speedAdjust, 200); // TODO: margin of error
  }

  public void test() {
    stage2_mainPIDController.setReference(SmartDashboard.getNumber("Stage 2 Velocity Setpoint", 0.0), ControlType.kVelocity);
    SmartDashboard.putNumber("Stage 2 speed", mainController.getEncoder().getVelocity());
  }

  /**
   * @return GREEN if everything is fine. RED if theres an error with the controllers
   */
  @Override
  public HealthState checkHealth() {
    if (isSparkMaxErrored(mainController) || isSparkMaxErrored(followerController)) {
      return HealthState.RED;
    }
    return HealthState.GREEN;
  }

  @Override
  public void mustangPeriodic() {
    // if (isShooting()){
    //   ballHasBeenShot = false;
    // } else if (!ballHasBeenShot && !isShooting()) {
    //   ballHasBeenShot = true;
    // }
    // SmartDashboard.putNumber("Stage 2 speed", mainController.getEncoder().getVelocity());
  }

  /**
   * 
   * @return if the shooter is currently shooting a ball
   */
  public boolean isShooting() {
    double current = mainController.getOutputCurrent();
    if (current > 0.2) {
      if (current >= NORMAL_CURRENT) {
        shootingCurrentCount++;
      } else {
        shootingCurrentCount = 0;
      }
      if (shootingCurrentCount >= 4) {
        return true;
      }
    }
    return false;
}

/**
 * 
 * @return if the ball has been shot
 */
  public boolean hasBallBeenShot() {
    return this.ballHasBeenShot;
  }
}