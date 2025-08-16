// TODO PUT COPYRIGHT YEAR!!! Copyright (c) <YEAR> FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static org.frc6423.lib.TestUtils.reset;
import static org.frc6423.lib.TestUtils.setupTest;

import org.junit.jupiter.api.BeforeEach;

public class RobotTest {
  /** Starts and closes the robot program */
  @BeforeEach
  public void setup() throws Exception {
    setupTest();
    new Robot().close();
    reset();
  }
}
