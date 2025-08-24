// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.arm;

/** Null {@link ArmIO} */
public class ArmIONone implements ArmIO {
  @Override
  public void periodic() {}

  @Override
  public double getPivotAngleRads() {
    return 0.0;
  }

  @Override
  public double getPivotSetpointAngleRads() {
    return 0.0;
  }

  @Override
  public double getPivotStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getRollerStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public void resetPivotEncoder(double poseRads) {}

  @Override
  public void runPivotVolts(double volts) {}

  @Override
  public void runPivotAngle(double angleRads) {}

  @Override
  public void runRollerVolts(double volts) {}

  @Override
  public void runRollerSpeed(double speedRpm) {}

  @Override
  public void close() throws Exception {}
}
