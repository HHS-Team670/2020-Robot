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
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.math.interpolable.InterpolatingDouble;
import frc.team670.robot.utils.math.interpolable.InterpolatingTreeMap;
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

  private double SPEED = 2500; // Will change later if we adjust by distance
  private static double DEFAULT_SPEED = 2500;

  private static double MAX_SHOT_DISTANCE_METERS = 8.382; // = 27.5 feet, this is a guess

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

  // Format: {Distance from target in meters, RPM}
  // Distance currently from bumper
  private static final double[][] FLYWHEEL_RPM_AT_DISTANCE = { 
    { 3.048, 2500 }, // baseline - 10 ft
    { 8.382, 2750 }, // longest shot we'll make - 27.5 ft
  };

  static {
    for (double[] pair : FLYWHEEL_RPM_AT_DISTANCE) {
      FLYWHEEL_RPM_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
    DEFAULT_SPEED = FLYWHEEL_RPM_MAP.getInterpolated(new InterpolatingDouble(MAX_SHOT_DISTANCE_METERS)).value;
  }

  private static final int VELOCITY_SLOT = 0;

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

  public double getStage2Velocity() {
    return stage2_mainEncoder.getVelocity();
  }

  public void run() {
    SmartDashboard.putNumber("Stage 2 speed", mainController.getEncoder().getVelocity());
    stage2_mainPIDController.setReference(SPEED, ControlType.kVelocity);
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

  public void setVelocityTarget(double targetRPM) {
    this.SPEED = targetRPM;
  }

  public double getTargetRPMForDistance(double distance){
    return 0;
    // TODO:
    // Find which known values in the table are closest to the distance we have
    // Interpolate: given that we know which values our distance is between, then similarly 
    // calculate what the target RPM should be. InterpolatingTreeMap should make a linear interpolation,
    // if time allows maybe make a quadratic model. 
    // Returns the found RPM
  }

  public void stop() {
    stage2_mainPIDController.setReference(0, ControlType.kDutyCycle);
  }

  public boolean isUpToSpeed() {
    return MathUtils.doublesEqual(getStage2Velocity(), SPEED, 200); // TODO: margin of error
  }

  public void test() {
    stage2_mainPIDController.setReference(SmartDashboard.getNumber("Stage 2 Velocity Setpoint", 0.0), ControlType.kVelocity);
    SmartDashboard.putNumber("Stage 2 speed", mainController.getEncoder().getVelocity());
  }

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
    SmartDashboard.putNumber("Stage 2 speed", mainController.getEncoder().getVelocity());
  }

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

  public boolean hasBallBeenShot() {
    return this.ballHasBeenShot;
  }
}