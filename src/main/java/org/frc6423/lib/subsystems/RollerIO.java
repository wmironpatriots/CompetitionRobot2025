// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.subsystems;

import edu.wpi.first.epilogue.Logged;

/** Generalized hardware methods for {@link Arm} subsystem */
public interface RollerIO extends AutoCloseable {
  /** Run periodic hardware logic */
  public void periodic();

  @Logged(name = "Roller Applied Voltage")
  public double getAppliedVolts();

  /**
   * @return speed of roller motor in revs per minute
   */
  @Logged(name = "Roller Motor Speed (Revs Per Minute)")
  public double getSpeedRpm();

  /**
   * @return stator current of roller motor in amps
   */
  @Logged(name = "Roller Motor Stator Current (Amps)")
  public double getStatorCurrentAmps();

  /**
   * Set roller motor voltage setpoint
   *
   * @param volts desired voltage setpoint
   */
  public void setVolts(double volts);

  /**
   * Set roller motor speed setpoint
   *
   * @param speedRpm desired speed setpoint in revs per minute
   */
  public void setSpeed(double speedRpm);
}
