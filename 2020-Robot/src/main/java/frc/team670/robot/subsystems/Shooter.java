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
import frc.team670.robot.utils.motorcontroller.SparkMAXFactory;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;
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
    stage1 = new VictorSPXLite(RobotMap.SHOOTER_STAGE_1);
    stage2Controllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SHOOTER_STAGE_2_MAIN, RobotMap.SHOOTER_STAGE_2_FOLLOWER, Motor_Type.NEO);

    stage2_mainController = stage2Controllers.get(0);
    stage2_followerController = stage2Controllers.get(1);

    stage2_mainEncoder = stage2_mainController.getEncoder();
    stage2_mainPIDController = stage2_mainController.getPIDController();

    SmartDashboard.putNumber("speed_Stage1", stage1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("P_Stage1", STAGE_1_V_P);
    SmartDashboard.putNumber("I_Stage1", STAGE_1_V_I);
    SmartDashboard.putNumber("D_Stage1", STAGE_1_V_D);
    SmartDashboard.putNumber("F_Stage1", STAGE_1_V_FF);
    setPIDStage1();

    SmartDashboard.putNumber("speed_Stage2", stage2_mainEncoder.getVelocity()); // Should display 2nd stage RPM
    SmartDashboard.putNumber("P_Stage2", STAGE_2_V_P);
    SmartDashboard.putNumber("I_Stage2", STAGE_2_V_I);
    SmartDashboard.putNumber("D_Stage2", STAGE_2_V_D);
    SmartDashboard.putNumber("F_Stage2", STAGE_2_V_FF);
    setPIDStage2();
  }

  public void stop() {
    stage2_mainController.set(0);
    stage1.set(ControlMode.PercentOutput, 0);
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

  public void setPIDStage1() {
    stage1.config_kP(0, SmartDashboard.getNumber("P_Stage1", 0.0));
    stage1.config_kI(0, SmartDashboard.getNumber("I_Stage1", 0.0));
    stage1.config_kD(0, SmartDashboard.getNumber("D_Stage1", 0.0));
    stage1.config_kF(0, SmartDashboard.getNumber("F_Stage1", 0.0));
  }

  public void setPIDStage2() {
    stage2_mainPIDController.setP(SmartDashboard.getNumber("P_Stage2", 0.0), STAGE_2_VELOCITY_SLOT);
    stage2_mainPIDController.setI(SmartDashboard.getNumber("I_Stage2", 0.0), STAGE_2_VELOCITY_SLOT);
    stage2_mainPIDController.setD(SmartDashboard.getNumber("D_Stage2", 0.0), STAGE_2_VELOCITY_SLOT);
    stage2_mainPIDController.setFF(SmartDashboard.getNumber("F_Stage2", 0.0), STAGE_2_VELOCITY_SLOT);
  }

  public void setSpeedStage1() {
    Logger.consoleLog("current-speed-Stage1 %s", SmartDashboard.getNumber("speed_Stage1", 0.0));
    stage1.set(ControlMode.PercentOutput, SmartDashboard.getNumber("speed_Stage1", 0.0));
  }

  public void setSpeedStage2() {
    Logger.consoleLog("Current stage 2 speed: %s", stage2_mainEncoder.getVelocity());
    double setPoint = SmartDashboard.getNumber("Set Velocity", 0);
    stage2_mainPIDController.setReference(setPoint, ControlType.kVelocity);
  }

  public void periodic() {
    setPIDStage1();
    setSpeedStage1();
    setPIDStage2();
    setSpeedStage1();
  }

  @Override
  public HealthState checkHealth() {
    // Can't use the shooter if stage 1 is dead
    ErrorCode stage1Error = stage1.getLastError();

    if (stage1Error != null && stage1Error != ErrorCode.OK) {
      return HealthState.RED;
    }
    if (isSparkMaxErrored(stage2_mainController) || isSparkMaxErrored(stage2_followerController)) {
      return HealthState.YELLOW;
    }
    return HealthState.GREEN;
  }

  @Override
  public void mustangPeriodic() {
    // TODO Auto-generated method stub

  }
}