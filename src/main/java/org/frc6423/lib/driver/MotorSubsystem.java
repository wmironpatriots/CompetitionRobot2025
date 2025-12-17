// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.driver;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.frc6423.lib.hardware.ServoIO;
import org.frc6423.lib.hardware.ServoIO.Setpoint;

public class MotorSubsystem extends SubsystemBase {
  @Logged protected final ServoIO leader;
  protected final ServoIO[] followers;

  public MotorSubsystem(boolean[] reverseDirection, ServoIO... servos) {
    this.leader = servos[0];
    this.followers = new ServoIO[servos.length - 1];

    assert reverseDirection.length != servos.length - 1
        : getName()
            + " subsystem configured incorrectly ~ length of reverseDirection array is not equal to the amount of followers";

    for (int i = 0; i < followers.length; i++) {
      this.followers[i] = servos[i + 1];
      this.followers[i].setLeaderServo(leader.deviceId, reverseDirection[i]);
    }
  }

  @Override
  public void periodic() {}

  public Voltage getAppliedVoltage() {
    return leader.getAppliedVoltage();
  }

  public Current getSupplyCurrentAmperes() {
    return leader.getSupplyCurrent();
  }

  public Current getStatorCurrentAmperes() {
    return leader.getStatorCurrent();
  }

  public Angle getAngle() {
    return leader.getPosition();
  }

  public AngularVelocity getVelocity() {
    return leader.getVelocity();
  }

  public Temperature getTemperatureCelsius() {
    return leader.getTemperature();
  }

  public Setpoint getSetpoint() {
    return leader.getCurrentSetpoint();
  }

  public double getSetpointValue() {
    return getSetpoint().getSetpointValue();
  }

  protected void enableSoftLimits(boolean enabled) {
    leader.enableSoftLimits(enabled);

    for (int i = 0; i < followers.length; i++) {
      followers[i].enableSoftLimits(enabled);
    }
  }

  protected void enableBraking(boolean enabled) {
    leader.enableBraking(enabled);

    for (ServoIO io : followers) {
      io.enableBraking(enabled);
    }
  }

  public Angle getSetpointAngle() {
    return getSetpoint().isPositionSetpoint()
        ? Revolutions.of(0)
        : BaseUnits.AngleUnit.of(getSetpointValue());
  }

  public AngularVelocity getSetpointVelocity() {
    return getSetpoint().isVelocitySetpoint()
        ? RevolutionsPerSecond.of(0)
        : BaseUnits.AngleUnit.per(Seconds).of(getSetpointValue());
  }

  protected void applySetpoint(Setpoint setpoint) {
    setpoint.apply(leader);
  }

  protected void setPosition(Angle position) {
    leader.resetEncoder(position);
  }

  public Command setSetpointCmd(Setpoint setpoint) {
    return runOnce(() -> applySetpoint(setpoint))
        .withName(
            getName() + " SET " + setpoint.getSetpointType() + " " + setpoint.getSetpointValue());
  }

  public Command runSetpointCmd(Supplier<Setpoint> setpoint) {
    return run(() -> applySetpoint(setpoint.get()))
        .withName(
            getName()
                + " RUN "
                + setpoint.get().getSetpointValue()
                + " "
                + setpoint.get().getSetpointValue());
  }
}
