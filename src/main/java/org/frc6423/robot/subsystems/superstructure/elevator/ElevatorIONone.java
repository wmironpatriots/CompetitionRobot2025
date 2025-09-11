// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

/** Null {@link ElevatorIO} */
public class ElevatorIONone implements ElevatorIO {
  public ElevatorIONone() {}

  @Override
  public void periodic() {}

  @Override
  public double getParentPoseMeters() {
    return 0.0;
  }

  @Override
  public double getChildPoseMeters() {
    return 0.0;
  }

  @Override
  public double getSetpointPoseMeters() {
    return 0.0;
  }

  @Override
  public double getVelocityMps() {
    return 0.0;
  }

  @Override
  public double getParentStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getChildStatorCurrentAmps() {
    return 0.0;
  }

  @Override
  public double getParentTempCelsius() {
    return 0.0;
  }

  @Override
  public double getChildTempCelsius() {
    return 0.0;
  }

  @Override
  public void resetEncoders(double poseMeters) {}

  @Override
  public void setGains(double kG, double kS, double kV, double kA, double kP, double kD) {}

  @Override
  public void setVolts(double volts) {}

  @Override
  public void setPose(double poseMeters) {}

  @Override
  public void close() throws Exception {}
}
