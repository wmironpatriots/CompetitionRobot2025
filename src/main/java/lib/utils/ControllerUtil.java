// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots 
// https://github.com/FIRSTTeam6423 
//
// Open Source Software; you can modify and/or share it under the terms of 
// MIT license file in the root directory of this project 
 
package lib.utils;

import edu.wpi.first.math.MathUtil;

public class ControllerUtil {
  /**
   * Squares and deadbands a joystick value
   *
   * @param val joystick value
   * @return modified joystick value
   */
  public static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.08);
  }
}
