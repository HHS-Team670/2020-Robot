/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.utils.motorcontroller.VictorSPXFactory;
import frc.team670.robot.utils.motorcontroller.VictorSPXLite;
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

  private double STAGE_1_SPEED = 0.3; // Change this later
  private double STAGE_2_SPEED = 0.5; // Percent output for testing (will use velocity setpoint later)

  private final double STAGE_2_PULLEY_RATIO = 2; // Need to check this

  private boolean ballHasBeenShot;
  private double stage2_current;
  private double stage2_prevCurrent;
  private double stage2_currentChange;

  private static final double SHOOTING_CURRENT_CHANGE_THRESHOLD = 0; // TODO: what is this?

  // Stage 1 Values (TODO: Tune)
  private static final double STAGE_1_V_P = 0;
  private static final double STAGE_1_V_I = 0;
  private static final double STAGE_1_V_D = 0;
  private static final double STAGE_1_V_FF = 0;

  private static final int STAGE_1_VELOCITY_SLOT = 0;

  // Practice Values Stage 2
  private static final double STAGE_2_V_P = 0.24;
  private static final double STAGE_2_V_I = 0.0;
  private static final double STAGE_2_V_D = 9.670;
  private static final double STAGE_2_V_FF = 0.033818;

  private static final int STAGE_2_VELOCITY_SLOT = 0;

  public Shooter() {

    SmartDashboard.putNumber("Stage 1 speed", 0.0);
    SmartDashboard.putNumber("Stage 2 speed", 0.0);

    // Stage 1 Victor should be inverted
    stage1 = VictorSPXFactory.buildFactoryVictorSPX(RobotMap.SHOOTER_STAGE_1, true);

    stage2Controllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SHOOTER_STAGE_2_MAIN,
        RobotMap.SHOOTER_STAGE_2_FOLLOWER, true, Motor_Type.NEO);

    stage2_mainController = stage2Controllers.get(0);
    stage2_followerController = stage2Controllers.get(1);

    stage2_mainEncoder = stage2_mainController.getEncoder();
    stage2_mainPIDController = stage2_mainController.getPIDController();
  }

  private double getStage2Velocity() {
    return stage2_mainEncoder.getVelocity() * STAGE_2_PULLEY_RATIO;
  }

  public void run() {
    stage1.set(ControlMode.PercentOutput, STAGE_1_SPEED);
    stage2_mainPIDController.setReference(STAGE_2_SPEED, ControlType.kVelocity);
  }

  public void stop() {
    stage2_mainController.set(0);
    stage1.set(ControlMode.PercentOutput, 0);
  }

  public boolean isUpToSpeed() {
    return MathUtils.doublesEqual(getStage2Velocity(), STAGE_2_SPEED, 0.05); // TODO: margin of error
  }

  public void setDefaultPID1() {
    stage1.config_kP(0, STAGE_1_V_P);
    stage1.config_kI(0, STAGE_1_V_I);
    stage1.config_kD(0, STAGE_1_V_D);
    stage1.config_kF(0, STAGE_1_V_FF);
  }

  public void setDefaultPID2() {
    stage2_mainController.getPIDController().setP(STAGE_2_V_P, STAGE_2_VELOCITY_SLOT);
    stage2_mainController.getPIDController().setI(STAGE_2_V_I, STAGE_2_VELOCITY_SLOT);
    stage2_mainController.getPIDController().setD(STAGE_2_V_D, STAGE_2_VELOCITY_SLOT);
    stage2_mainController.getPIDController().setFF(STAGE_2_V_FF, STAGE_2_VELOCITY_SLOT);
  }

  public void test() {

    stage1.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Stage 1 speed", 0.0));

    stage2_mainController.set(SmartDashboard.getNumber("Stage 2 speed", 0.0));

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