// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;

/** Generalized hardware methods for {@link ArmPivot} subsystem */
public interface ArmPivotIO extends AutoCloseable {
  /** Run periodic hardware logic */
  public void periodic();

  /**
   * @return angle of pivot motor in radians
   */
  @Logged(name = "Pivot Motor Angle (Radians)")
  public double getAngleRads();

  /**
   * @return setpoint angle of pivot motor in radians
   */
  @Logged(name = "Pivot Motor Setpoint Angle (Radians)")
  public double getSetpointAngleRads();

  /**
   * @return stator current of pivot motor in amps
   */
  @Logged(name = "Pivot Motor Stator Current (Amps)")
  public double getStatorCurrentAmps();

  /**
   * Reset relative encoder to specified position
   *
   * @param poseRads position to reset to in radians
   */
  public void resetEncoder(double poseRads);

  /**
   * Set pivot motor voltage setpoint
   *
   * @param volts desired voltage setpoint
   */
  public void setVolts(double volts);

  /**
   * Set pivot motor angle setpoint
   *
   * @param angleRads desired angle setpoint in radians
   */
  public void setAngle(double angleRads);
}
