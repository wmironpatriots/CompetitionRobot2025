// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/** Pivot subsystem-component of the {@link Arm} Subsystem */
public class ArmPivot extends SubsystemBase implements AutoCloseable {
  /** Name of the CAN bus hardware is on */
  public static final String CANBUS = "RIO";

  /** Pivot motor CAN ID */
  public static final int MOTOR_ID = 14;

  /** Gear ratio of the pivot gearbox */
  public static final double GEAR_REDUCTION = 50;

  /** Moment of Inertia of the arm around the pivot */
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.23381);

  /** Length of the arm */
  public static final Distance LENGTH = Inches.of(7.5);

  /** The smallest feasible angle */
  public static final Angle MIN_ANGLE = Degrees.of(-90);

  /** The largest feasible angle */
  public static final Angle MAX_ANGLE = Degrees.of(90);

  /** The max allowable angle error */
  public static final Angle TOLERANCE = Degrees.of(1.5);

  /** The velocity limit of the arm's trapezoid profile */
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(5.5);

  /** The acceleration of the arm's trapezoid profile */
  public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(17);

  @Logged(name = "Arm Pivot Hardware Loggables")
  private final ArmPivotIO hardware;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  @Logged(name = "Filted Pivot Motor Stator Current (Amps)")
  private double filteredCurrent;

  private boolean isZeroed = false;

  public ArmPivot(ArmPivotIO hardware) {
    this.hardware = hardware;
  }

  @Override
  public void periodic() {
    hardware.periodic();

    filteredCurrent = currentFilter.calculate(hardware.getStatorCurrentAmps());
  }

  /**
   * @return true if arm has been homed
   */
  @Logged(name = "Is Zeroed (bool)")
  public boolean isZeroed() {
    return isZeroed;
  }

  /**
   * @return true if arm is within a certain tolerance of the setpoint angle
   */
  @Logged(name = "Near Setpoint Angle (bool)")
  public boolean isNearSetpointAngle() {
    return MathUtil.isNear(
        hardware.getSetpointAngleRads(), hardware.getAngleRads(), TOLERANCE.in(Radians));
  }

  /**
   * @return {@link Rotation2d} representing the pivot angle
   */
  @Logged(name = "Angle (Rotation2d)")
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(hardware.getAngleRads());
  }

  /**
   * Run arm into hardstop to determine home (aka, zero)
   *
   * @return {@link Command}
   */
  // TODO CHECK VALUES
  public Command runCurrentHoming() {
    return this.run(() -> hardware.setVolts(-2.5))
        .until(() -> Math.abs(filteredCurrent) > 50.0)
        .finallyDo(
            (interupted) -> {
              if (!interupted) {
                hardware.resetEncoder(0.0);
                isZeroed = true;
              }
            });
  }

  /**
   * Run arm to specified angle
   *
   * @param angle {@link Angle} representing desired angle
   * @return {@link Command}
   */
  public Command runAngle(Angle angle) {
    return this.run(() -> hardware.setAngle(angle.in(Radians)));
  }

  /**
   * Run arm to specified angle
   *
   * @param angle desired angle in radians
   * @return {@link Command}
   */
  public Command runAngle(DoubleSupplier angleRads) {
    return this.run(() -> hardware.setAngle(angleRads.getAsDouble()));
  }

  /**
   * Run arm to specified angle
   *
   * @param angle desired angle in radians
   * @return {@link Command}
   */
  public Command runAngle(double angleRads) {
    return this.runAngle(() -> angleRads);
  }

  /**
   * Hold pivot at current angle
   *
   * @return {@link Command}
   */
  public Command holdAngle() {
    return Commands.sequence(
        this.runAngle(hardware.getAngleRads()).until(() -> true), this.run(() -> {}));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
