// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import org.frc6423.lib.util.TestUtil;
import org.junit.jupiter.api.Test;

public class RobotTest {
  @Test
  void initalize() throws Exception {
    new Robot().close();
    TestUtil.reset();
  }
}
