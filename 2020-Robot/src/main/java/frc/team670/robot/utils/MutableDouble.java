/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils;

/**
 * Represents a mutable double
 */
public class MutableDouble {

  private double value;

  /**
   * Used to create a mutable double
   * @param value the value of the double
   */
  public MutableDouble(double value) {
    this.value = value;
  }

  /**
   * Used to get the value of the mutable double
   * @return
   */
  public double getValue() {
    return this.value;
  }

  /**
   * Used to set the value of the mutable double
   * @param value the new value of the double
   */
  public void setValue(double value) {
    this.value = value;
  }
}