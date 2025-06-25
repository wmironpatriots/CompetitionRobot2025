// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots 
// https://github.com/wmironpatriots 
//
// Open Source Software; you can modify and/or share it under the terms of 
// MIT license file in the root directory of this project 
 
package wmironpatriots.subsystems.swerve.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import lib.utils.CanDeviceId;

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
