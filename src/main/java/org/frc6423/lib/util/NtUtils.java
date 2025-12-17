// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.util;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Utilities for {@link NetworkTables} */
public class NtUtils {
  /**
   * Create a new {@link IntegerEntry}
   *
   * @param path {@link String} representing topic path
   * @param defaultValue default entry value
   * @return {@link DoubleEntry}
   */
  public static IntegerEntry createIntegerEntry(String path, int defaultValue) {
    var entry = NetworkTableInstance.getDefault().getIntegerTopic(path).getEntry(defaultValue);
    entry.set(defaultValue);

    return entry;
  }

  /**
   * Create a new {@link DoubleEntry}
   *
   * @param path {@link String} representing topic path
   * @param defaultValue default entry value
   * @return {@link DoubleEntry}
   */
  public static DoubleEntry createDoubleEntry(String path, double defaultValue) {
    var entry = NetworkTableInstance.getDefault().getDoubleTopic(path).getEntry(defaultValue);
    entry.set(defaultValue);

    return entry;
  }

  /**
   * Create a new {@link BooleanEntry}
   *
   * @param path {@link String} representing topic path
   * @param defaultValue default entry value
   * @return {@link BooleanEntry}
   */
  public static BooleanEntry createBooleanEntry(String path, boolean defaultValue) {
    var entry = NetworkTableInstance.getDefault().getBooleanTopic(path).getEntry(defaultValue);
    entry.set(defaultValue);

    return entry;
  }

  /**
   * Create a new {@link DoubleArrayEntry}
   *
   * @param path {@link String} representing topic path
   * @param defaultValue default entry value
   * @return {@link DoubleArrayEntry}
   */
  public static DoubleArrayEntry createDoubleArrayEntry(String path, double[] defaultValue) {
    var entry = NetworkTableInstance.getDefault().getDoubleArrayTopic(path).getEntry(defaultValue);
    entry.set(defaultValue);

    return entry;
  }
}
