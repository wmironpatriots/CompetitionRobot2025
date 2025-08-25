// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;

/** Generalized hardware methods for {@link Arm} subsystem */
public interface ArmIO extends AutoCloseable {
  /** Run periodic hardware logic */
  public void periodic();

  /**
   * @return angle of pivot motor in radians
   */
  @Logged(name = "Pivot Motor Angle (Radians)")
  public double getPivotAngleRads();

  /**
   * @return setpoint angle of pivot motor in radians
   */
  @Logged(name = "Pivot Motor Setpoint Angle (Radians)")
  public double getPivotSetpointAngleRads();

  /**
   * @return stator current of pivot motor
   */
  @Logged(name = "Pivot Motor Stator Current (Amps)")
  public double getPivotStatorCurrentAmps();

  /**
   * @return speed of roller motor in revs per minute
   */
  @Logged(name = "Roller Motor Speed (Revs Per Minute)")
  public double getRollerSpeedRpm();

  /**
   * @return stator current of roller motor
   */
  @Logged(name = "Roller Motor Stator Current (Amps)")
  public double getRollerStatorCurrentAmps();

  /**
   * Reset pivot motor relative encoder to specified position
   *
   * @param poseRads position to reset to
   */
  public void resetPivotEncoder(double poseRads);

  /**
   * Run pivot motor at specified voltage setpoint
   *
   * @param volts desired voltage
   */
  public void runPivotVolts(double volts);

  /**
   * Run pivot motor to specified angle using onboard closed-loop control
   *
   * @param angleRads desired position in radians
   */
  public void runPivotAngle(double angleRads);

  /**
   * Run roller motor at specified voltage setpoint
   *
   * @param volts desired voltage
   */
  public void runRollerVolts(double volts);

  /**
   * Run roller motor at specific speed using onboard open-loop control
   *
   * @param speedRpm desired speed in revs per minute
   */
  public void runRollerSpeed(double speedRpm);
}
