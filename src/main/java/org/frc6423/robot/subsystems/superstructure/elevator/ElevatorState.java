// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/** Represents an extension height the {@link Elevator} subsystem can be in */
public enum ElevatorState {
  /** Resting state */
  STOWED(Meters.of(0.0)),
  /** L2 height */
  L2(Inches.of(4.549)),
  /** L3 height */
  L3(Inches.of(12.41)),
  /** L4 height */
  L4(Inches.of(24.0));

  public final Distance height;

  private ElevatorState(Distance height) {
    this.height = height;
  }
}
