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
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.MAX_ANGLE;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.MIN_ANGLE;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.stream.Stream;
import org.frc6423.robot.subsystems.superstructure.arm.Arm;
import org.frc6423.robot.subsystems.superstructure.arm.Arm.ArmAngle;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/** {@link Arm} subsystem test class */
public class ArmTests {
  /** The max time a test can run for before auto failing */
  public static final Time TIMEOUT = Seconds.of(3.0);

  private Arm arm;
  private Timer timer = new Timer();

  @BeforeEach
  public void init() {
    // Init sys
    setupTest();
    // Create arm
    arm = Arm.create();
    // Start test timer
    timer.start();
  }

  @AfterEach
  public void destroy() throws Exception {
    // Stop and record test time
    timer.stop();
    System.out.println("FINISHED TEST IN " + timer.get() + " SECONDS");

    // Reset elevator
    reset(arm);
  }

  /** Run 10 randomly generated angles between the min and max angle */
  @RepeatedTest(10)
  public void runRandAngle() {
    runToCompletion(
        arm.runAngle(MAX_ANGLE.minus(MIN_ANGLE).times(Math.random()).plus(MIN_ANGLE))
            .until(arm::isNearSetpointAngle)
            .withTimeout(TIMEOUT));
  }

  /**
   * Run all {@link ArmAngle}
   *
   * @param angle {@link ArmAngle}
   */
  @ParameterizedTest
  @MethodSource("provideAngles")
  public void runAngles(ArmAngle angle) {
    runToCompletion(arm.runAngle(angle).until(arm::isNearSetpointAngle).withTimeout(TIMEOUT));
  }

  /**
   * @return {@link Stream} of all {@link ArmAngle}
   */
  private static Stream<Arguments> provideAngles() {
    ArrayList<Arguments> angles = new ArrayList<>();
    for (var angle : ArmAngle.values()) {
      angles.add(Arguments.of(angle));
    }

    return angles.stream();
  }
}
