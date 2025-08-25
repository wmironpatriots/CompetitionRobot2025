// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

/** Represents a state the {@link Arm} subsystem can be in */
public enum ArmState {
  /** Resting state */
  STOWED(Rotation2d.fromDegrees(0.0), 0.0),
  /** Avoidance state for preventing collisions */
  AVOIDING(Rotation2d.fromDegrees(0.0), 0.0),
  /** Flipped state for intaking */
  INTAKING(Rotation2d.fromDegrees(0.0), 0.0),
  /** L2 pose but not scoring */
  L2_PRIMED(Rotation2d.fromDegrees(0.0), 0.0),
  /** L3 pose but not scoring */
  L3_PRIMED(Rotation2d.fromDegrees(0.0), 0.0),
  /** L4 pose but not scoring */
  L4_PRIMED(Rotation2d.fromDegrees(0.0), 0.0),
  /** L2 scoring */
  L2_SCORING(Rotation2d.fromDegrees(0.0), 0.0),
  /** L3 scoring */
  L3_SCORING(Rotation2d.fromDegrees(0.0), 0.0),
  /** L4 scoring */
  L4_SCORING(Rotation2d.fromDegrees(0.0), 0.0);

  public final Rotation2d angle;
  public final double rollerSpeedRpm;

  private ArmState(Rotation2d angle, double rollerSpeedRpm) {
    this.angle = angle;
    this.rollerSpeedRpm = rollerSpeedRpm;
  }
}
