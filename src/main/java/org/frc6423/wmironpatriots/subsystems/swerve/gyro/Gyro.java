// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.wmironpatriots.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/** Generalized Hardware methods for a gyro */
public abstract class Gyro {
  /**
   * @return {@link Rotation2d} representing the gyro heading
   */
  public abstract Rotation2d getRotation2d();

  /**
   * Reset gyro to specified heading
   *
   * @param heading {@link Rotation2d} representing the heading to reset to
   */
  public abstract void reset(Rotation2d heading);
}
