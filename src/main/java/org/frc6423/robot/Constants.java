// TODO PUT COPYRIGHT YEAR!!! Copyright (c) <YEAR> FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

/**
 * Stores global immutable values called "constants"
 *
 * <p>When creating a new constant, make sure it is static and final
 */
public class Constants {
  /** Constants that affect robot behavior during runtime */
  public class Flags {
    public static final Time LOOPTIME = Seconds.of(0.02);
  }

  /** Constants representing port IDs that devices are connected to */
  public class Ports {}
}
