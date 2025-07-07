// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.wmironpatriots.subsystems.swerve.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc6423.lib.utils.CanDeviceId;

public class GyroPigeon extends Gyro {
  private final Pigeon2 pigeon;

  public GyroPigeon(CanDeviceId pigeonId) {
    pigeon = new Pigeon2(pigeonId.getCanId(), pigeonId.getBusName());
  }

  @Override
  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  @Override
  public void reset(Rotation2d heading) {
    pigeon.reset();
  }
}
