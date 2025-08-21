// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.types;

public class CanDeviceId {
  private final String busName;
  private final int canId;

  public CanDeviceId(String busName, int canId) {
    this.busName = busName;
    this.canId = canId;
  }

  public String getBusName() {
    return busName;
  }

  public int getCanId() {
    return canId;
  }
}
