// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;

public class ServoIOKrakenFoc extends ServoIO {
  private final TalonFX servo;
  private TalonFXConfiguration config;

  private final BaseStatusSignal voltSig, supplySig, statorSig, poseSig, velSig, tempSig;

  private final DutyCycleOut dutycycleReq = new DutyCycleOut(0.0).withEnableFOC(true);
  private final VoltageOut voltReq = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC poseReq = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velReq = new VelocityTorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC profPoseReq = new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC profVelReq =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
  private ThreadPoolExecutor threadPoolExecutor =
      new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

  public ServoIOKrakenFoc(String canBusId, int canDeviceId, TalonFXConfiguration config) {
    super(canBusId, canDeviceId);
    servo = new TalonFX(canDeviceId, canBusId);
    this.config = config;
    applyConfig(config);

    voltSig = servo.getMotorVoltage();
    supplySig = servo.getSupplyCurrent();
    statorSig = servo.getStatorCurrent();
    poseSig = servo.getPosition();
    velSig = servo.getVelocity();
    tempSig = servo.getDeviceTemp();
  }

  public void applyConfig(TalonFXConfiguration config) {
    threadPoolExecutor.submit(
        () -> {
          for (int i = 0; i < 5; i++) {
            StatusCode result = servo.getConfigurator().apply(config);
            if (result.isOK()) {
              break;
            }
          }
        });
  }

  @Override
  public Voltage getAppliedVoltage() {
    return Volts.of(voltSig.getValueAsDouble());
  }

  @Override
  public Current getSupplyCurrent() {
    return Amps.of(supplySig.getValueAsDouble());
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(statorSig.getValueAsDouble());
  }

  @Override
  public Angle getPosition() {
    return Revolutions.of(poseSig.getValueAsDouble());
  }

  @Override
  public AngularVelocity getVelocity() {
    return RevolutionsPerSecond.of(velSig.getValueAsDouble());
  }

  @Override
  public Temperature getTemperature() {
    return Celsius.of(tempSig.getValueAsDouble());
  }

  @Override
  protected void idle() {
    servo.stopMotor();
  }

  @Override
  protected void setDutyCycleSetpoint(Dimensionless dutycycle) {
    servo.setControl(dutycycleReq.withOutput(dutycycle.baseUnitMagnitude()));
  }

  @Override
  protected void setVoltageSetpoint(Voltage voltage) {
    servo.setControl(voltReq.withOutput(voltage));
  }

  @Override
  protected void setCurrentSetpoint(Current current) {
    servo.setControl(currentReq.withOutput(current));
  }

  @Override
  protected void setPositionSetpoint(Angle position, int slot) {
    servo.setControl(poseReq.withPosition(position).withSlot(slot));
  }

  @Override
  protected void setVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
    servo.setControl(velReq.withVelocity(velocity).withAcceleration(acceleration).withSlot(slot));
  }

  @Override
  protected void setProfiledPositionSetpoint(Angle position, int slot) {
    servo.setControl(profPoseReq.withPosition(position).withSlot(slot));
  }

  @Override
  protected void setProfiledVelocitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
    servo.setControl(
        profVelReq.withVelocity(velocity).withAcceleration(acceleration).withSlot(slot));
  }

  @Override
  public void resetEncoder(Angle position) {
    threadPoolExecutor.submit(() -> servo.setPosition(position));
  }

  @Override
  public void setLeaderServo(int canDeviceId, boolean reverseDirection) {
    servo.setControl(new Follower(canDeviceId, reverseDirection));
  }

  @Override
  public void enableSoftLimits(boolean enabled) {
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
  }

  @Override
  public void enableBraking(boolean enabled) {
    threadPoolExecutor.submit(
        () -> servo.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast));
  }
}
