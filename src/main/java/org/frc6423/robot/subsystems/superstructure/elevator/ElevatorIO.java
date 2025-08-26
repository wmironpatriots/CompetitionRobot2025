// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

import edu.wpi.first.epilogue.Logged;

/** Generalized hardware methods for {@link Elevator} subsystem */
public interface ElevatorIO extends AutoCloseable {
  /** Run periodic hardware logic */
  public void periodic();

  /**
   * @return parent motor position in meters
   */
  @Logged(name = "Parent Motor Position (Meters)")
  public double getParentPoseMeters();

  /**
   * @return child motor position in meters
   */
  @Logged(name = "Child Motor Position (Meters)")
  public double getChildPoseMeters();

  /**
   * @return child/parent motor setpoint position
   */
  @Logged(name = "Parent and Child Motors Setpoint Position (Meters)")
  public double getSetpointPoseMeters();

  /**
   * @return parent motor stator current in amps
   */
  @Logged(name = "Parent Motor Stator Current (Amps)")
  public double getParentStatorCurrentAmps();

  /**
   * @return child motor stator current in amps
   */
  @Logged(name = "Child Motor Stator Current (Amps)")
  public double getChildStatorCurrentAmps();

  /**
   * @return parent motor temperature in celsius
   */
  @Logged(name = "Parent Motor Temperature (Celsius)")
  public double getParentTempCelsius();

  /**
   * @return child motor temperature in celsius
   */
  @Logged(name = "Child Motor Temperature (Celsius)")
  public double getChildTempCelsius();

  /**
   * Reset relative encoders of parent/child motor to specified position
   *
   * @param poseMeters position to reset to in radians
   */
  public void resetEncoders(double poseMeters);

  /**
   * Set parent/child motors voltage setpoint
   *
   * @param volts desired voltage setpoint
   */
  public void setVolts(double volts);

  /**
   * Set parent/child motors position setpoint /w acceleration setpoint
   *
   * @param poseMeters desired position setpoint in meters
   * @param accelerationMpsSqrd desired acceleration setpoint in meters per second per second
   */
  public void setPose(double poseMeters, double accelerationMpsSqrd);
}
