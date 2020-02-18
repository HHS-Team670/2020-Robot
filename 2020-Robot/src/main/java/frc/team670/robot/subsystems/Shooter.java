/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.math.InterpolatingDouble;
import frc.team670.robot.utils.math.InterpolatingTreeMap;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.VictorSPXFactory;
import frc.team670.robot.utils.motorcontroller.MotorConfig.Motor_Type;

/**
 * Represents a 2-stage shooter, with 1st stage using a VictorSPX and 2-NEO 2nd
 * stage with SparkMax controllers.
 * 
 * @author ctychen, Pallavi Das
 */
public class Shooter extends MustangSubsystemBase {

  private VictorSPX stage1;
  private SparkMAXLite stage2_mainController, stage2_followerController;
  private List<SparkMAXLite> stage2Controllers;

  private CANEncoder stage2_mainEncoder;
  private CANPIDController stage2_mainPIDController;

  private static double STAGE_1_SPEED = 0.3; // Change this later
  private double STAGE_2_SPEED = 2750; // Will change later if we adjust by distance
  private static double STAGE_2_DEFAULT_SPEED;

  private static double MAX_SHOT_DISTANCE_METERS = 8.382; // = 27.5 feet, this is a guess

  private static final double STAGE_2_PULLEY_RATIO = 2; // Need to check this

  private boolean ballHasBeenShot;
  private double stage2_current;
  private double stage2_prevCurrent;
  private double stage2_currentChange;

  private static final double SHOOTING_CURRENT_CHANGE_THRESHOLD = 0; // TODO: what is this?

  // Stage 2 values, as of 2/17 testing
  private static final double STAGE_2_V_P = 0.000100;
  private static final double STAGE_2_V_I = 0.0;
  private static final double STAGE_2_V_D = 0.0;
  private static final double STAGE_2_V_FF = 0.000183;
  private static final double STAGE_2_RAMP_RATE = 1.0;

  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> FLYWHEEL_RPM_MAP = new InterpolatingTreeMap<>();

  // Format: {Distance from target in meters, Stage 2 RPM}
  // All a guess right now
  private static final double[][] FLYWHEEL_RPM_AT_DISTANCE = { 
    { 8.382, 2750 }

  };

  static {
    for (double[] pair : FLYWHEEL_RPM_AT_DISTANCE) {
      FLYWHEEL_RPM_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
    STAGE_2_DEFAULT_SPEED = FLYWHEEL_RPM_MAP.getInterpolated(new InterpolatingDouble(MAX_SHOT_DISTANCE_METERS)).value;
  }

  private static final int STAGE_2_VELOCITY_SLOT = 0;

  public Shooter() {

    SmartDashboard.putNumber("Stage 1 speed", 0.0);
    SmartDashboard.putNumber("Stage 2 Velocity Setpoint", 0.0);
    SmartDashboard.putNumber("Stage 2 FF", 0.0);
    SmartDashboard.putNumber("Stage 2 P", 0.0);
    SmartDashboard.putNumber("Stage 2 Ramp Rate", 0.0);

    // Stage 1 Victor should be inverted
    stage1 = VictorSPXFactory.buildFactoryVictorSPX(RobotMap.SHOOTER_STAGE_1, true);

    stage2Controllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SHOOTER_STAGE_2_MAIN,
        RobotMap.SHOOTER_STAGE_2_FOLLOWER, true, Motor_Type.NEO);

    stage2_mainController = stage2Controllers.get(0);
    stage2_followerController = stage2Controllers.get(1);

    stage2_mainEncoder = stage2_mainController.getEncoder();
    stage2_mainPIDController = stage2_mainController.getPIDController();

    stage2_mainPIDController.setP(STAGE_2_V_P, STAGE_2_VELOCITY_SLOT);
    stage2_mainPIDController.setI(STAGE_2_V_I, STAGE_2_VELOCITY_SLOT);
    stage2_mainPIDController.setD(STAGE_2_V_D, STAGE_2_VELOCITY_SLOT);
    stage2_mainPIDController.setFF(STAGE_2_V_FF, STAGE_2_VELOCITY_SLOT);
  }

  private double getStage2Velocity() {
    return stage2_mainEncoder.getVelocity() * STAGE_2_PULLEY_RATIO;
  }

  public void run() {
    stage1.set(ControlMode.PercentOutput, STAGE_1_SPEED);
    stage2_mainPIDController.setReference(STAGE_2_SPEED, ControlType.kVelocity);
  }

  /**
   * @param setRamp true if we want a ramp rate (use this for getting the shooter
   *                up to speed), false when we're ready to shoot and don't need
   *                one
   */
  public void setRampRate(boolean setRamp) {
    if (setRamp) {
      stage2_mainController.setClosedLoopRampRate(STAGE_2_RAMP_RATE);
    } else {
      stage2_mainController.setClosedLoopRampRate(0);
    }
  }

  public void setVelocityTarget(double targetRPM) {
    this.STAGE_2_SPEED = targetRPM;
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
    stage2_mainController.set(0);
    stage1.set(ControlMode.PercentOutput, 0);
  }

  public boolean isUpToSpeed() {
    return MathUtils.doublesEqual(getStage2Velocity(), STAGE_2_SPEED, 10); // TODO: margin of error
  }

  public void test() {

    stage1.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Stage 1 speed", 0.0));

    stage2_mainPIDController.setReference(SmartDashboard.getNumber("Stage 2 Velocity Setpoint", 0.0),
        ControlType.kVelocity);

    setRampRate(true);

    SmartDashboard.putNumber("Stage 2 RPM", stage2_mainController.getEncoder().getVelocity());
  }

  @Override
  public HealthState checkHealth() {
    // Can't use the shooter if stage 1 is dead
    if (isPhoenixControllerErrored(stage1)) {
      return HealthState.RED;
    }
    if (isSparkMaxErrored(stage2_mainController) || isSparkMaxErrored(stage2_followerController)) {
      return HealthState.YELLOW;
    }
    return HealthState.GREEN;
  }

  @Override
  public void mustangPeriodic() {
    stage2_prevCurrent = stage2_current;
    stage2_current = stage2_mainController.getOutputCurrent();
    stage2_currentChange = stage2_current - stage2_prevCurrent;
    if (stage2_currentChange > SHOOTING_CURRENT_CHANGE_THRESHOLD) {
      ballHasBeenShot = true;
    } else {
      ballHasBeenShot = false;
    }
  }

  public boolean hasBallBeenShot() {
    return this.ballHasBeenShot;
  }
}