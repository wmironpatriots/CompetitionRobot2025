// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Static methods for building tests
 *
 * <p>Most methods are "Borrowed" from FRC 1155 (luv u guys)
 */
public class TestUtil {
  /** Set up DS and initalize HAL */
  public static void setupTest() {
    setupTest(AllianceStationID.Blue1);
  }

  /** Set up DS with specified alliance station and initalize HAL */
  public static void setupTest(AllianceStationID allianceStationId) {
    assert HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(allianceStationId);
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

  /**
   * Runs CommandScheduler and updates timer repeatedly to fast forward subsystems and run commands.
   *
   * @param ticks The number of times CommandScheduler is run
   */
  public static void fastForward(int ticks) {
    for (int i = 0; i < ticks; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(0.02);
    }
  }

  /**
   * Fasts forward in time by running CommandScheduler and updating timer.
   *
   * @param time
   */
  public static void fastForward(Time time) {
    fastForward((int) (time.in(Seconds) / 0.02));
  }

  /**
   * Runs CommandScheduler and updates timer to fast forward subsystems by 4 seconds and run
   * commands.
   */
  public static void fastForward() {
    fastForward(Seconds.of(4));
  }

  /**
   * Schedules and runs a command.
   *
   * @param command The command to run.
   */
  public static void run(Command command) {
    command.schedule();
    CommandScheduler.getInstance().run();
  }

  /**
   * Schedules and runs a command.
   *
   * @param command The command to run.
   * @param runs The number of times CommandScheduler is run.
   */
  public static void run(Command command, int runs) {
    command.schedule();
    fastForward(runs);
  }

  /**
   * Schedules a command and runs it until it ends. Be careful -- if the command you give never
   * ends, this will be an infinite loop!
   *
   * @param command The command to run
   */
  public static void runToCompletion(Command command) {
    run(command, 1);
    while (command.isScheduled()) {
      fastForward();
    }
  }
}
