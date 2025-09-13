// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import edu.wpi.first.epilogue.Logged;

/** Generalized hardware methods for {@link Arm} subsystem */
public interface ArmIO extends AutoCloseable {
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
   * @return pivot motor velocity in Radians/Second
   */
  @Logged(name = "Pivot Motor Velocity (Radians Per Second)")
  public double getVelocityRadsPerSec();

  /**
   * @return stator current of pivot motor in amps
   */
  @Logged(name = "Pivot Motor Stator Current (Amps)")
  public double getStatorCurrentAmps();

  /**
   * Reset relative encoder of pivot motor to specified position
   *
   * @param poseRads position to reset to in radians
   */
  public void resetEncoder(double poseRads);

  /**
   * Set arm gains for onboard feedforward and feedback control
   *
   * @param kG output to overcome gravity
   * @param kS output to overcome static friction
   * @param kV output per unit of target velocity (output/mps)
   * @param kA output per unit of target acceleration (output/(mps/s))
   * @param kP output per unit of error in position (meters)
   * @param kD output per unit of error in velocity (mps)
   * @param maxVel max velocity of motion profile
   * @param maxAccel max acceleration of motion profile
   */
  public void setGains(
      double kG,
      double kS,
      double kV,
      double kA,
      double kP,
      double kD,
      double maxVel,
      double maxAccel);

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
