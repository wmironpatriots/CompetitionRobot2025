// TODO PUT COPYRIGHT YEAR!!! Copyright (c) <YEAR> FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Contains static methods for creating tests */
public class TestUtils {
  /** Set up DS and initalizes HAL with default values and asserts that it doesn't fail */
  public static void setupTest() {
    assert HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setTest(true);
    DriverStationSim.notifyNewData();
    SimHooks.restartTiming();
  }

  /**
   * Reset {@link CommandScheduler} and closes all subsystems
   *
   * @param subsystems all subsystems to be closed
   */
  public static void reset(AutoCloseable... subsystems) throws Exception {
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().cancelAll();

    for (AutoCloseable subsystem : subsystems) {
      subsystem.close();
    }
  }
}
