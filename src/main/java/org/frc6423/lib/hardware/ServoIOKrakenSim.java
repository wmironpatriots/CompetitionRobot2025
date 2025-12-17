// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.hardware;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ServoIOKrakenSim extends ServoIO {
  public ServoIOKrakenSim(String canBusId, int deviceId) {
    super(canBusId, deviceId);
  }

  @Override
  public Voltage getAppliedVoltage() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAppliedVoltage'");
  }

  @Override
  public Current getSupplyCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSupplyCurrent'");
  }

  @Override
  public Current getStatorCurrent() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getStatorCurrent'");
  }

  @Override
  public Angle getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  @Override
  public AngularVelocity getVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
  }

  @Override
  public Temperature getTemperature() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTemperature'");
  }

  @Override
  protected void idle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'idle'");
  }

  @Override
  protected void setDutyCycleSetpoint(Dimensionless setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDutyCycleSetpoint'");
  }

  @Override
  protected void setVoltageSetpoint(Voltage setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltageSetpoint'");
  }

  @Override
  protected void setCurrentSetpoint(Current setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentSetpoint'");
  }

  @Override
  protected void setPositionSetpoint(Angle setpoint, int slot) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPositionSetpoint'");
  }

  @Override
  protected void setVelocitySetpoint(
      AngularVelocity setpoint, AngularAcceleration acceleration, int slot) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVelocitySetpoint'");
  }

  @Override
  protected void setProfiledPositionSetpoint(Angle setpoint, int slot) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setProfiledPositionSetpoint'");
  }

  @Override
  protected void setProfiledVelocitySetpoint(
      AngularVelocity setpoint, AngularAcceleration acceleration, int slot) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setProfiledVelocitySetpoint'");
  }

  @Override
  public void resetEncoder(Angle setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetEncoder'");
  }

  @Override
  public void setLeaderServo(int canDeviceId, boolean reverseDirection) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setLeaderServo'");
  }

  @Override
  public void enableSoftLimits(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'enableSoftLimits'");
  }

  @Override
  public void enableBraking(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'brakeEnabled'");
  }
}
