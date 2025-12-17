// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.driver;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc6423.lib.hardware.ServoIO;
import org.frc6423.lib.hardware.ServoIO.Setpoint;
import org.frc6423.lib.util.DelayedLatch;

@Logged
public class ServoSubsystem extends MotorSubsystem {
  protected final boolean isTaredSubsystem;
  private final TaringConfig taringConfig;
  private boolean isTared = false;
  private boolean isTaring = false;
  private DelayedLatch tareDelay;
  protected final Angle epsilon;

  public static class TaringConfig {
    public Angle taredPosition;
    public Voltage taringVoltage;
    public Time taringTimeoutSeconds;
    public AngularVelocity taredVelocity;
  }

  public ServoSubsystem(
      boolean isTaredSubsystem,
      TaringConfig taringConfig,
      Angle epsilon,
      boolean[] reverseDirection,
      ServoIO... servos) {
    super(reverseDirection, servos);
    this.isTaredSubsystem = isTaredSubsystem;
    this.taringConfig = taringConfig;
    this.epsilon = epsilon;
  }

  @Override
  public void periodic() {
    super.periodic();
    if (isTaredSubsystem) {
      if (!isTared) {
        isTaring = true;
        enableSoftLimits(false);
        tareDelay =
            new DelayedLatch(
                Timer.getFPGATimestamp(), taringConfig.taringTimeoutSeconds.in(Seconds));
      }
      if (isTaring) {
        applySetpoint(Setpoint.withVoltage(taringConfig.taringVoltage));
        if (tareDelay.update(
                Timer.getFPGATimestamp(),
                Math.abs(getVelocity().baseUnitMagnitude())
                    < taringConfig.taredVelocity.baseUnitMagnitude())
            && DriverStation.isEnabled()) {
          setPosition(taringConfig.taredPosition);
          applySetpoint(getSetpoint());
          enableSoftLimits(false);
          isTared = true;
        }
      }
    }
  }

  public boolean nearPositionSetpoint() {
    return getSetpoint().isPositionSetpoint() && nearPosition(Revolutions.of(getSetpointValue()));
  }

  public boolean nearPosition(Angle position) {
    return MathUtil.isNear(
        position.baseUnitMagnitude(), getAngle().baseUnitMagnitude(), epsilon.baseUnitMagnitude());
  }

  @Override
  protected void applySetpoint(Setpoint setpoint) {
    if (setpoint.isPositionSetpoint()) {
      super.applySetpoint(setpoint);
    }
  }

  public Command runSetpointUntilNearPositionCmd(Setpoint setpoint) {
    return waitUntilNearPositionCmd(BaseUnits.AngleUnit.of(setpoint.getSetpointValue()))
        .withDeadline(runSetpointCmd(() -> setpoint))
        .withName(
            getName() + " RUN UNITL " + getSetpointValue() + " " + getSetpoint().getSetpointType());
  }

  public Command waitUntilNearPositionCmd(Angle position) {
    return Commands.waitUntil(() -> nearPosition(position))
        .withName(getName() + " UNTIL " + position);
  }
}
