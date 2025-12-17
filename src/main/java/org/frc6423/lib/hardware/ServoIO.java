// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.UnaryOperator;

public abstract class ServoIO implements Sendable {
  public static enum SetpointType {
    IDLE,
    DUTYCYCLE,
    VOLTAGE,
    CURRENT,
    POSITION,
    VELOCITY,
    PROFILED_POSITION,
    PROFILED_VELOCITY;
  }

  public static class Setpoint {
    private final double value;
    private final SetpointType type;
    private final UnaryOperator<ServoIO> applier;

    private Setpoint(double value, SetpointType mode, UnaryOperator<ServoIO> applier) {
      this.value = value;
      this.type = mode;
      this.applier = applier;
    }

    public double getSetpointValue() {
      return value;
    }

    public SetpointType getSetpointType() {
      return type;
    }

    public boolean isPositionSetpoint() {
      return type == SetpointType.POSITION || type == SetpointType.PROFILED_POSITION;
    }

    public boolean isVelocitySetpoint() {
      return type == SetpointType.VELOCITY || type == SetpointType.VELOCITY;
    }

    public static Setpoint withDutycycle(Dimensionless output) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setDutyCycleSetpoint(output);
            return io;
          };
      return new Setpoint(output.baseUnitMagnitude(), SetpointType.DUTYCYCLE, applier);
    }

    public static Setpoint withVoltage(Voltage output) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setVoltageSetpoint(output);
            return io;
          };
      return new Setpoint(output.baseUnitMagnitude(), SetpointType.VOLTAGE, applier);
    }

    public static Setpoint withCurrent(Current output) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setCurrentSetpoint(output);
            return io;
          };
      return new Setpoint(output.baseUnitMagnitude(), SetpointType.CURRENT, applier);
    }

    public static Setpoint withPosition(Angle output) {
      return withPosition(output, 0);
    }

    public static Setpoint withPosition(Angle output, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setPositionSetpoint(output, slot);
            return io;
          };
      return new Setpoint(output.baseUnitMagnitude(), SetpointType.POSITION, applier);
    }

    public static Setpoint withVelocity(AngularVelocity output) {
      return withVelocity(output, RadiansPerSecondPerSecond.of(0.0), 0);
    }

    public static Setpoint withVelocity(AngularVelocity output, int slot) {
      return withVelocity(output, RadiansPerSecondPerSecond.of(0.0), slot);
    }

    public static Setpoint withVelocity(AngularVelocity output, AngularAcceleration acceleration) {
      return withVelocity(output, acceleration, 0);
    }

    public static Setpoint withVelocity(
        AngularVelocity output, AngularAcceleration acceleration, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setVelocitySetpoint(output, acceleration, slot);
            return io;
          };
      return new Setpoint(output.baseUnitMagnitude(), SetpointType.VELOCITY, applier);
    }

    public static Setpoint withProfiledPosition(Angle output) {
      return withProfiledPosition(output, 0);
    }

    public static Setpoint withProfiledPosition(Angle output, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setProfiledPositionSetpoint(output, slot);
            return io;
          };
      return new Setpoint(output.baseUnitMagnitude(), SetpointType.PROFILED_POSITION, applier);
    }

    public static Setpoint withProfiledVelocity(AngularVelocity output) {
      return withProfiledVelocity(output, RadiansPerSecondPerSecond.of(0.0), 0);
    }

    public static Setpoint withProfiledVelocity(AngularVelocity output, int slot) {
      return withProfiledVelocity(output, RadiansPerSecondPerSecond.of(0), slot);
    }

    public static Setpoint withProfiledVelocity(
        AngularVelocity output, AngularAcceleration acceleration) {
      return withProfiledVelocity(output, acceleration, 0);
    }

    public static Setpoint withProfiledVelocity(
        AngularVelocity output, AngularAcceleration acceleration, int slot) {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.setProfiledVelocitySetpoint(output, acceleration, slot);
            return io;
          };
      return new Setpoint(output.baseUnitMagnitude(), SetpointType.PROFILED_VELOCITY, applier);
    }

    public static Setpoint idle() {
      UnaryOperator<ServoIO> applier =
          (ServoIO io) -> {
            io.idle();
            return io;
          };
      return new Setpoint(0.0, SetpointType.IDLE, applier);
    }

    public void apply(ServoIO io) {
      applier.apply(io);
    }
  }

  private Setpoint currentSetpoint = Setpoint.idle();

  public final String canBusId;
  public final int deviceId;

  public ServoIO(String canBusId, int deviceId) {
    this.canBusId = canBusId;
    this.deviceId = deviceId;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Applied Voltage", () -> getAppliedVoltage().in(Volts), null);
    builder.addDoubleProperty("Supply Current (Ampere)", () -> getSupplyCurrent().in(Amps), null);
    builder.addDoubleProperty("Stator Current (Ampere)", () -> getStatorCurrent().in(Amps), null);

    builder.addDoubleProperty("Position (Revolution)", () -> getPosition().in(Revolutions), null);
    builder.addDoubleProperty(
        "Velocity (Revolutions Per Second)", () -> getVelocity().in(RevolutionsPerSecond), null);

    builder.addDoubleProperty("Temperature (Celsius)", () -> getTemperature().in(Celsius), null);

    builder.addStringProperty(
        "Setpoint Type", () -> getCurrentSetpoint().getSetpointType().toString(), null);
    builder.addDoubleProperty("Setpoint", () -> getCurrentSetpoint().getSetpointValue(), null);
  }

  public abstract Voltage getAppliedVoltage();

  public abstract Current getSupplyCurrent();

  public abstract Current getStatorCurrent();

  public abstract Angle getPosition();

  public abstract AngularVelocity getVelocity();

  public abstract Temperature getTemperature();

  public Setpoint getCurrentSetpoint() {
    return currentSetpoint;
  }

  protected abstract void idle();

  protected abstract void setDutyCycleSetpoint(Dimensionless setpoint);

  protected abstract void setVoltageSetpoint(Voltage setpoint);

  protected abstract void setCurrentSetpoint(Current setpoint);

  protected abstract void setPositionSetpoint(Angle setpoint, int slot);

  protected abstract void setVelocitySetpoint(
      AngularVelocity setpoint, AngularAcceleration acceleration, int slot);

  protected abstract void setProfiledPositionSetpoint(Angle setpoint, int slot);

  protected abstract void setProfiledVelocitySetpoint(
      AngularVelocity setpoint, AngularAcceleration acceleration, int slot);

  public abstract void resetEncoder(Angle setpoint);

  public abstract void setLeaderServo(int canDeviceId, boolean reverseDirection);

  public abstract void enableSoftLimits(boolean enabled);

  /**
   * Enable/disable brake mode
   *
   * <p>Brake mode affects how the servo slows down when no power is applied:
   *
   * <ul>
   *   <li>Enabled: The servo short its terminals to deaccelerate quicker
   *   <li>Disabled: The servo will apply no form of braking and slowly deaccelerate
   * </ul>
   *
   * By default, brake mode is enabled
   *
   * <p>! WARNING ! When brake is disabled, the servo will slowly deaccelerate which can make it a
   * safety risk
   *
   * @param enabled set true to enable brake, false to disable
   */
  public abstract void enableBraking(boolean enabled);
}
