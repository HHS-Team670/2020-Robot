/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.team670.robot.commands.drive.teleop.XboxRocketLeagueDrive;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.NavX;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Represents a tank drive base.
 * 
 * @author lakshbhambhani, ctychen
 */
public class DriveBase extends SubsystemBase {

  private CANSparkMax left1, left2, right1, right2;
  private CANEncoder left1Encoder, left2Encoder, right1Encoder, right2Encoder;

  private SpeedControllerGroup left, right;
  private DifferentialDrive driveTrain;

  private List<CANSparkMax> leftControllers, rightControllers;
  private List<CANSparkMax> allMotors = new ArrayList<CANSparkMax>();;

  private NavX navXMicro;
  private DifferentialDriveOdometry m_odometry;

  private static final double sparkMaxVelocityConversionFactor = RobotConstants.DRIVEBASE_METERS_PER_ROTATION / 60;

  public DriveBase() {
    left1 = new CANSparkMax(RobotMap.SPARK_LEFT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    left2 = new CANSparkMax(RobotMap.SPARK_LEFT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    right1 = new CANSparkMax(RobotMap.SPARK_RIGHT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    right2 = new CANSparkMax(RobotMap.SPARK_RIGHT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);

    left1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();

    left1Encoder = left1.getEncoder();
    right1Encoder = right1.getEncoder();
    left2Encoder = left2.getEncoder();
    right2Encoder = right2.getEncoder();
    
    left1Encoder.setVelocityConversionFactor(sparkMaxVelocityConversionFactor);
    left2Encoder.setVelocityConversionFactor(sparkMaxVelocityConversionFactor); // Do not invert for right side
    right1Encoder.setVelocityConversionFactor(sparkMaxVelocityConversionFactor);
    right2Encoder.setVelocityConversionFactor(sparkMaxVelocityConversionFactor);

    leftControllers = Arrays.asList(left1, left2);
    rightControllers = Arrays.asList(right1, right2);
    allMotors.addAll(leftControllers);
    allMotors.addAll(rightControllers);

    setMotorsInvert(leftControllers, false);
    setMotorsInvert(rightControllers, true); // Invert this so it will work properly with the CANPIDController and then invert the SpeedController to compensate for automatic inversion.

    left2.follow(left1);
    right2.follow(right1);

    left = new SpeedControllerGroup(left1, left2);
    right = new SpeedControllerGroup(right1, right2);
    
    // The DifferentialDrive inverts the right side automatically, however we want
    // to invert straight from the Spark so that we can
    // still use it properly with the CANPIDController, so we need to reverse the
    // automatic invert.
    right.setInverted(true);

    driveTrain = new DifferentialDrive(left, right);
    driveTrain.setMaxOutput(1.0);

    setRampRate(allMotors, 0.36); // Will automatically cook some Cheezy Poofs

    // initialized NavX and sets Odometry
    navXMicro = new NavX(RobotMap.NAVX_PORT);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
      new Pose2d(0, 0, new Rotation2d()));

  }


  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton).
   * Squares inputs to linearize them.
   * 
   * @param leftSpeed  Speed for left side of drive base [-1, 1]. Automatically
   *                   squares this value to linearize it.
   * @param rightSpeed Speed for right side of drive base [-1, 1]. Automatically
   *                   squares this value to linearize it.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton)
   * 
   * @param leftSpeed     Speed for left side of drive base [-1, 1]
   * @param rightSpeed    Speed for right side of drive base [-1, 1]
   * @param squaredInputs If true, decreases sensitivity at lower inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
    driveTrain.tankDrive(leftSpeed, rightSpeed, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using a curvature drive configuration (wheel)
   * 
   * @param xSpeed      The forward throttle speed [-1, 1]
   * @param zRotation   The amount of rotation to turn [-1, 1] with positive being
   *                    right
   * @param isQuickTurn If true enables turning in place and running one side
   *                    backwards to turn faster
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with
   * twist)
   * 
   * @param xSpeed      The forward throttle speed [-1, 1]
   * @param zRotation   The amount of rotation to turn [-1, 1] with positive being
   *                    right
   * @param isQuickTurn If true, decreases sensitivity at lower inputs
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
    driveTrain.arcadeDrive(xSpeed, zRotation, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with
   * twist)
   * 
   * @param xSpeed    The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being
   *                  right
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    arcadeDrive(xSpeed, zRotation, false);
  }

  /**
   * Stops the motors on the drive base (sets them to 0).
   */
  public void stop() {
    tankDrive(0, 0);
  }

  /**
   * Sets all motors to Brake Mode
   */
  public void initBrakeMode() {
    setMotorsBrakeMode(allMotors, IdleMode.kBrake);
  }

  /*
   * Gets the input voltage of all the motor controllers on the robot
   */
  public double getRobotInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of all the motor controllers on the robot
   */
  public double getRobotOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput() + right1.getAppliedOutput()
        + right2.getAppliedOutput();
    return output;
  }

  /**
   * Gets the encoder position of the front left motor in ticks.
   */
  public int getLeftSparkEncoderPosition() {
    return (int) (left1.getEncoder().getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
  }

  /**
   * Gets the encoder position of the front right motor in ticks.
   */
  public int getRightSparkEncoderPosition() {
    return (int) (right1.getEncoder().getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
  }

  /**
   * Inverts a list of motors.
   */
  private void setMotorsInvert(List<CANSparkMax> motorGroup, boolean invert) {
    for (CANSparkMax m : motorGroup) {
      m.setInverted(invert);
    }
  }

  /**
   * Sets array of motors to be of a specified mode
   */
  public void setMotorsNeutralMode(IdleMode mode) {
    for (CANSparkMax m : allMotors) {
      m.setIdleMode(mode);
    }
  }

  /**
   * Sets array of motor to coast mode
   */
  public void setMotorsCoastMode(List<CANSparkMax> motorGroup, IdleMode mode) {
    for (CANSparkMax m : motorGroup) {
      m.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Sets array of motor to brake mode
   */
  public void setMotorsBrakeMode(List<CANSparkMax> motorGroup, IdleMode mode) {
    for (CANSparkMax m : motorGroup) {
      m.setIdleMode(IdleMode.kBrake);
    }
  }

  /*
   * Gets the voltage fed into the motor controllers on the left side of the robot
   */
  public double getLeftInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage();
    return output;
  }

  /*
   * Get the voltage fed into the motor controllers on the right side of the robot
   */
  public double getRightInputVoltage() {
    double output = right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of the motor controllers on the left side of the
   * robot
   */
  public double getLeftOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output voltage of the motor controllers on the right side of the
   * robot
   */
  public double getRightOutputVoltage() {
    double output = right1.getAppliedOutput() + right2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the left side
   * of the robot
   */
  public double getLeftOutputCurrent() {
    double output = left1.getOutputCurrent() + left2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the right side
   * of the robot
   */
  public double getRightOutputCurrent() {
    double output = right1.getOutputCurrent() + right2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the input voltage of all the drivebase motor controllers on the robot
   */
  public double getDriveBaseInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of all the drivebase motor controllers on the robot
   */
  public double getDriveBaseOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput() + right1.getAppliedOutput()
        + right2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output current of all the motor controllers on the robot
   */
  public double getRobotOutputCurrent() {
    double output = left1.getOutputCurrent() + left2.getOutputCurrent() + right1.getOutputCurrent()
        + right2.getOutputCurrent();
    return output;
  }

  public void initDefaultCommand() {
    CommandScheduler.getInstance().schedule(new XboxRocketLeagueDrive(this));
  }

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the Spark Encoder
   */
  public double getLeftSparkEncoderVelocityInches() {
    return (DriveBase.convertDriveBaseTicksToInches(
        left1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
  }

  /**
   * Returns the velocity of the right side of the drivebase in ticks/second from
   * the Spark Encoder
   */
  public double getLeftSparkEncoderVelocityTicks() {
    return (left1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
  }

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the Spark Encoder
   */
  public double getRightSparkEncoderVelocityInches() {
    return (DriveBase.convertDriveBaseTicksToInches(
        right1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
  }

  /**
   * Returns the velocity of the right side of the drivebase in ticks/second from
   * the Spark Encoder
   */
  public double getRightSparkEncoderVelocityTicks() {
    return (right1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
  }

  /**
   * Returns the Spark Max Encoder for the Left Main Motor
   */
  public CANEncoder getLeftMainEncoder(){
    return left1Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Left Follower Motor
   */
  public CANEncoder getLeftFollowerEncoder(){
    return left2Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Right Main Motor
   */
  public CANEncoder getRightMainEncoder(){
    return right1Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Right Follower Motor
   */
  public CANEncoder getRightFollowerEncoder(){
    return right2Encoder;
  }

  /**
   * Returns the Left Motor Controllers
   * @return The list of the motor controller on the left side of the robot
   */
  public List<CANSparkMax> getLeftControllers() {
    return leftControllers;
  }

  /**
   * Returns the Right Motor Controller
   * @return The list of hte motor controller on the right side of the robot
   */
  public List<CANSparkMax> getRightControllers() {
    return rightControllers;
  }

  /**
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setRampRate(List<CANSparkMax> motors, double rampRate) {
    for (CANSparkMax m : motors) {
      m.setClosedLoopRampRate(rampRate);
      m.setOpenLoopRampRate(rampRate);
    }
  }

  /**
   * Converts a tick value taken from a drive base DIO encoder to inches.
   */
  public static double convertDriveBaseTicksToInches(double ticks) {
    double rotations = ticks / RobotConstants.DIO_TICKS_PER_ROTATION;
    return rotations * Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER;
  }

  public static double convertSparkRevolutionsToInches(double revolutions) {
    // rev * 2piR in / rev
    return revolutions * Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER / RobotConstants.DRIVEBASE_GEAR_RATIO;
  }

  /**
   * Converts an inch value into drive base DIO Encoder ticks.
   */
  public static int convertInchesToDriveBaseTicks(double inches) {
    double rotations = inches / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
    return (int) (rotations * RobotConstants.DIO_TICKS_PER_ROTATION);
  }

  /**
   * Gets inches per rotations of a NEO motor on the drive base since SparkMAX
   * encoders work in rotations.
   */
  public static double convertDriveBaseRotationsToInches(double rotations) {
    return RobotConstants.DRIVEBASE_INCHES_PER_ROTATION * rotations;
  }

  /**
   * Gets rotations of a NEO motor on the drive base per a value in inches ince
   * SparkMAX encoders work in rotations.
   */
  public static double convertInchesToDriveBaseRotations(double inches) {
    return inches / RobotConstants.DRIVEBASE_INCHES_PER_ROTATION;
  }

  /**
   * Converts a value of per second of the DriveBase Rounds Per Minute
   */
  public static double convertInchesPerSecondToDriveBaseRoundsPerMinute(double inchesPerSecond) {
    // (Inches/seconds) * (60 seconds/1 minute) * ((2 * Diameter inches)/Rotation)
    return inchesPerSecond * 60 / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
  }

  public void initCoastMode() {
    setMotorsNeutralMode(IdleMode.kCoast);
  }

  public void sendEncoderDataToDashboard() {
    SmartDashboard.putNumber("Left M Position Ticks", left1Encoder.getPosition());
    SmartDashboard.putNumber("Left M Velocity Ticks", left1Encoder.getVelocity());
    SmartDashboard.putNumber("Left S Position Ticks", left2Encoder.getPosition());
    SmartDashboard.putNumber("Left S Velocity Ticks", left2Encoder.getVelocity());
    SmartDashboard.putNumber("Right M Position Ticks", right1Encoder.getPosition());
    SmartDashboard.putNumber("Right M Velocity Ticks", right1Encoder.getVelocity());
    SmartDashboard.putNumber("Right S Position Ticks", right2Encoder.getPosition());
    SmartDashboard.putNumber("Right S Velocity Ticks", right2Encoder.getVelocity());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), left1Encoder.getPosition(), right1Encoder.getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void resetOdometry() {
    zeroHeading();
    left1Encoder.setPosition(0);
    right1Encoder.setPosition(0);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
        new Pose2d(0, 0, new Rotation2d()));
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navXMicro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navXMicro.getAngle(), 360) * (RobotConstants.kNavXReversed ? -1. : 1.);
  }

  /**
   * Returns the wheel speeds of the leftMain Motor and rightMainMotor
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left1Encoder.getVelocity(), right1Encoder.getVelocity());
  }

  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    tankDrive(leftVoltage / RobotController.getBatteryVoltage(), rightVoltage / RobotController.getBatteryVoltage());
  }

}