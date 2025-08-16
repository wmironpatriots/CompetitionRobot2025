// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Time;
import org.frc6423.lib.CanDeviceId;

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
  public class Ports {
    // * CANIVORE LOOP
    public static final CANBus CANCHAN = new CANBus("CANchan"); // :3
    public static final CanDeviceId PIGEON = new CanDeviceId(CANCHAN.getName(), 0);
    public static final CanDeviceId BL_PIVOT = new CanDeviceId(CANCHAN.getName(), 1);
    public static final CanDeviceId BL_DRIVE = new CanDeviceId(CANCHAN.getName(), 2);
    public static final CanDeviceId FL_PIVOT = new CanDeviceId(CANCHAN.getName(), 3);
    public static final CanDeviceId FL_DRIVE = new CanDeviceId(CANCHAN.getName(), 4);
    public static final CanDeviceId FR_PIVOT = new CanDeviceId(CANCHAN.getName(), 5);
    public static final CanDeviceId FR_DRIVE = new CanDeviceId(CANCHAN.getName(), 6);
    public static final CanDeviceId BR_PIVOT = new CanDeviceId(CANCHAN.getName(), 7);
    public static final CanDeviceId BR_DRIVE = new CanDeviceId(CANCHAN.getName(), 8);
    public static final CanDeviceId BL_CANCODER = new CanDeviceId(CANCHAN.getName(), 9);
    public static final CanDeviceId FL_CANCODER = new CanDeviceId(CANCHAN.getName(), 10);
    public static final CanDeviceId FR_CANCODER = new CanDeviceId(CANCHAN.getName(), 11);
    public static final CanDeviceId BR_CANCODER = new CanDeviceId(CANCHAN.getName(), 12);
    public static final CanDeviceId ELEVATOR_PARENT = new CanDeviceId(CANCHAN.getName(), 14);
    public static final CanDeviceId ELEVATOR_CHILD = new CanDeviceId(CANCHAN.getName(), 15);

    // * RIO LOOP
    public static final CANBus RIO = new CANBus("rio");
    public static final CanDeviceId TAIL_ROLLER = new CanDeviceId(RIO.getName(), 1);
    public static final CanDeviceId CHUTE_ROLLER = new CanDeviceId(RIO.getName(), 2);
    public static final CanDeviceId TAIL_PIVOT = new CanDeviceId(RIO.getName(), 13);
  }
}
