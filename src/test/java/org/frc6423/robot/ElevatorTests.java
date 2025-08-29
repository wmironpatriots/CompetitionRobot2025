// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Seconds;
import static org.frc6423.lib.utilities.TestUtils.reset;
import static org.frc6423.lib.utilities.TestUtils.runToCompletion;
import static org.frc6423.lib.utilities.TestUtils.setupTest;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.MAX_EXTENSION_HEIGHT;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.stream.Stream;
import org.frc6423.robot.subsystems.superstructure.elevator.Elevator;
import org.frc6423.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import org.frc6423.robot.subsystems.superstructure.elevator.ElevatorState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/** {@link Elevator} subsystem test class */
class ElevatorTests {
  /** The max time a test can run for before auto failing */
  static final Time TIMEOUT = Seconds.of(3.0);

  Elevator elevator;
  Timer timer = new Timer();

  @BeforeEach
  public void init() {
    // Init sys
    setupTest();
    // Create elevator
    elevator = new Elevator(new ElevatorIOSim());
    // Start test timer
    timer.start();
  }

  @AfterEach
  public void destroy() throws Exception {
    // Stop and record test time
    timer.stop();
    System.out.println("FINISHED TEST IN " + timer.get() + " SECONDS");

    // Reset elevator
    reset(elevator);
  }

  /** Run 10 randomly generated extension heights between 0.0 and the max extension height */
  @RepeatedTest(10)
  public void runRandExtension() {
    runToCompletion(
        elevator
            // Picks a random extension height between 0.0 and max extension height
            .runExtension(MAX_EXTENSION_HEIGHT.times(Math.random()))
            .until(() -> elevator.isNearSetpointPose())
            .withTimeout(TIMEOUT));
  }

  /**
   * Run all {@link ElevatorState}
   *
   * @param extension {@link ElevatorState}
   */
  @ParameterizedTest
  @MethodSource("provideExtensionHeights")
  public void runExtensions(ElevatorState extension) {
    runToCompletion(
        elevator
            .runExtension(extension)
            .until(() -> elevator.isNearSetpointPose())
            .withTimeout(TIMEOUT));
  }

  /**
   * @return {@link Stream} of all {@link ElevatorState}
   */
  private static Stream<Arguments> provideExtensionHeights() {
    ArrayList<Arguments> extensions = new ArrayList<>();
    for (var extension : ElevatorState.values()) {
      extensions.add(Arguments.of(extension));
    }

    return extensions.stream();
  }
}
