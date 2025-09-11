// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static org.frc6423.lib.utilities.CtreUtils.*;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.CANBUS;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.CHILD_MOTOR_ID;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.MAX_ACCELERATION;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.MAX_EXTENSION_HEIGHT;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.MAX_VELOCITY;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.PARENT_MOTOR_ID;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.SENSOR_TO_MECH_RATIO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Comp bot {@link ElevatorIO} */
public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX parent = new TalonFX(PARENT_MOTOR_ID, CANBUS);
  private final TalonFX child = new TalonFX(CHILD_MOTOR_ID, CANBUS);

  private final TalonFXConfiguration conf = new TalonFXConfiguration();

  private final VoltageOut voltReq = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC poseReq = new MotionMagicTorqueCurrentFOC(0.0);

  private final BaseStatusSignal parentPoseSig, parentCurrentSig, parentTempSig;
  private final BaseStatusSignal childPoseSig, childCurrentSig, childTempSig;

  public ElevatorIOReal() {
    conf.Audio.BeepOnBoot = true;
    conf.Audio.BeepOnConfig = true;

    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    conf.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    conf.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = MAX_EXTENSION_HEIGHT.in(Meters);

    conf.CurrentLimits.SupplyCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLimit = 40.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;
    conf.CurrentLimits.StatorCurrentLimit = 80.0;

    conf.Feedback.SensorToMechanismRatio = SENSOR_TO_MECH_RATIO;
    conf.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

    conf.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    conf.Slot0.kG = 0.0;
    conf.Slot0.kS = 0.0;
    conf.Slot0.kV = 0.0;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kP = 0.0;
    conf.Slot0.kI = 0.0;
    conf.Slot0.kD = 0.0;

    conf.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.in(MetersPerSecond);
    conf.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.in(MetersPerSecondPerSecond);

    tryUntilOk(() -> parent.getConfigurator().apply(conf), 10, PARENT_MOTOR_ID);
    tryUntilOk(() -> child.getConfigurator().apply(conf), 10, CHILD_MOTOR_ID);
    child.setControl(new Follower(parent.getDeviceID(), true));

    parentPoseSig = parent.getPosition();
    parentCurrentSig = parent.getStatorCurrent();
    parentTempSig = parent.getDeviceTemp();

    childPoseSig = child.getPosition();
    childCurrentSig = child.getStatorCurrent();
    childTempSig = child.getDeviceTemp();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        parentPoseSig,
        parentCurrentSig,
        parentTempSig,
        childPoseSig,
        childCurrentSig,
        childTempSig);
  }

  @Override
  public double getParentPoseMeters() {
    return parentPoseSig.getValueAsDouble();
  }

  @Override
  public double getChildPoseMeters() {
    return childPoseSig.getValueAsDouble();
  }

  @Override
  public double getSetpointPoseMeters() {
    return poseReq.Position;
  }

  @Override
  public double getParentStatorCurrentAmps() {
    return parentCurrentSig.getValueAsDouble();
  }

  @Override
  public double getChildStatorCurrentAmps() {
    return childCurrentSig.getValueAsDouble();
  }

  @Override
  public double getParentTempCelsius() {
    return parentTempSig.getValueAsDouble();
  }

  @Override
  public double getChildTempCelsius() {
    return childTempSig.getValueAsDouble();
  }

  @Override
  public void resetEncoders(double poseMeters) {
    parent.setPosition(poseMeters);
  }

  @Override
  public void setGains(double kG, double kS, double kV, double kA, double kP, double kD) {
    conf.Slot0.kG = kG;
    conf.Slot0.kS = kS;
    conf.Slot0.kV = kV;
    conf.Slot0.kA = kA;
    conf.Slot0.kP = kP;
    conf.Slot0.kD = kD;

    // I think the configurator will only apply if the config is different so this should be fine
    tryUntilOk(() -> parent.getConfigurator().apply(conf), 10, PARENT_MOTOR_ID);
    tryUntilOk(() -> child.getConfigurator().apply(conf), 10, CHILD_MOTOR_ID);
  }

  @Override
  public void setVolts(double volts) {
    parent.setControl(voltReq.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPose(double poseMeters) {
    parent.setControl(poseReq.withPosition(poseMeters));
  }

  @Override
  public void close() throws Exception {
    parent.close();
    child.close();
  }
}
