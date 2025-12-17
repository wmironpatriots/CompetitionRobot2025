// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.util;

/** An iterative boolean latch that delays the transition from false to true. */
public class DelayedLatch {
  private boolean currentValue;
  private double lastTimestamp;
  private final double delay;

  public DelayedLatch(double timestamp, double delay) {
    lastTimestamp = timestamp;
    currentValue = false;
    this.delay = delay;
  }

  public boolean update(double timestamp, boolean value) {
    boolean result = false;

    if (value && !currentValue) {
      lastTimestamp = timestamp;
    }

    // If we are still true and we have transitioned.
    if (value && (timestamp - lastTimestamp > delay)) {
      result = true;
    }

    currentValue = value;
    return result;
  }
}
