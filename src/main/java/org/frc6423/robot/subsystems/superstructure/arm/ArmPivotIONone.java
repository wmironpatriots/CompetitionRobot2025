// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

/** Null {@link ArmPivotIO} */
public class ArmPivotIONone implements ArmPivotIO {
  public ArmPivotIONone() {}

  @Override
  public void periodic() {}

  @Override
  public double getAngleRads() {
    return 0.0;
  }

  @Override
  public double getSetpointAngleRads() {
    return 0.0;
  }

  @Override
  public double getStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public void resetEncoder(double poseRads) {}

  @Override
  public void setVolts(double volts) {}

  @Override
  public void setAngle(double angleRads) {}

  @Override
  public void close() throws Exception {}
}
