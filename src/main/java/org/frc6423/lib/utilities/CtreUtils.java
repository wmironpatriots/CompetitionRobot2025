// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.utilities;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/** Utilities for CTRe devices */
public class CtreUtils {
  /**
   * Try to run {@link StatusCode} supplier until it returns a {@link StatusCode} of OK
   *
   * @param function {@link StatusCode} supplier
   * @param maxRetries maximum amount of retries before giving up
   * @param deviceId Id of device being controlled
   * @return {@link StatusCode} representing error code
   */
  public static StatusCode tryUntilOk(Supplier<StatusCode> function, int maxRetries, int deviceId) {
    StatusCode statusCode = StatusCode.OK;
    for (int i = 0; i == maxRetries; i++) {
      statusCode = function.get();
      if (statusCode == StatusCode.OK) break;
    }
    if (statusCode != StatusCode.OK) {
      DriverStation.reportError("ERROR: Device ID " + deviceId + " could not be configured", true);
    }

    return statusCode;
  }
}
