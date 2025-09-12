// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static org.frc6423.lib.utilities.CtreUtils.tryUntilOk;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.CANBUS;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.GEAR_REDUCTION;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.MAX_ACCELERATION;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.MAX_ANGLE;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.MAX_VELOCITY;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.MIN_ANGLE;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.MOTOR_ID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmIOReal implements ArmIO {
  private final TalonFX motor = new TalonFX(MOTOR_ID, CANBUS);

  private final TalonFXConfiguration conf = new TalonFXConfiguration();

  private final VoltageOut voltReq = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC poseReq = new MotionMagicTorqueCurrentFOC(0.0);

  private final BaseStatusSignal poseSig, currentSig;

  public ArmIOReal() {
    conf.Audio.BeepOnBoot = true;
    conf.Audio.BeepOnConfig = true;

    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    conf.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    conf.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = MAX_ANGLE.in(Radians);
    conf.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    conf.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = MIN_ANGLE.in(Radians);

    conf.CurrentLimits.SupplyCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLimit = 40.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;
    conf.CurrentLimits.StatorCurrentLimit = 80.0;

    conf.Feedback.SensorToMechanismRatio = GEAR_REDUCTION;
    conf.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

    conf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    conf.Slot0.kG = 0.0;
    conf.Slot0.kS = 0.0;
    conf.Slot0.kV = 0.0;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kP = 0.0;
    conf.Slot0.kI = 0.0;
    conf.Slot0.kD = 0.0;

    conf.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.in(RadiansPerSecond);
    conf.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.in(RadiansPerSecondPerSecond);

    tryUntilOk(() -> motor.getConfigurator().apply(conf), 10, MOTOR_ID);

    poseSig = motor.getPosition();
    currentSig = motor.getStatorCurrent();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(poseSig, currentSig);
  }

  @Override
  public double getAngleRads() {
    return poseSig.getValueAsDouble();
  }

  @Override
  public double getSetpointAngleRads() {
    return poseReq.getPositionMeasure().in(Radians);
  }

  @Override
  public double getStatorCurrentAmps() {
    return currentSig.getValueAsDouble();
  }

  @Override
  public void resetEncoder(double poseRads) {
    motor.setPosition(poseRads);
  }

  @Override
  public void setGains(
      double kG,
      double kS,
      double kV,
      double kA,
      double kP,
      double kD,
      double maxVel,
      double maxAccel) {

    conf.Slot0.kG = kG;
    conf.Slot0.kS = kS;
    conf.Slot0.kV = kV;
    conf.Slot0.kA = kA;
    conf.Slot0.kP = kP;
    conf.Slot0.kD = kD;
    conf.MotionMagic.MotionMagicCruiseVelocity = maxVel;
    conf.MotionMagic.MotionMagicAcceleration = maxAccel;

    // I think the configurator will only apply if the config is different so this should be fine
    tryUntilOk(() -> motor.getConfigurator().apply(conf), 10, MOTOR_ID);
  }

  @Override
  public void setVolts(double volts) {
    motor.setControl(voltReq.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setAngle(double angleRads) {
    motor.setControl(poseReq.withPosition(angleRads));
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
