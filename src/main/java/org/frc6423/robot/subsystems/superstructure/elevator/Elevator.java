// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/** Elevator Subsystem */
public class Elevator extends SubsystemBase {
  /** CONSTANTS */
  // TODO
  public static final double GEAR_RATIO = 0.0;

  // TODO
  public static final Distance MAX_EXTENSION_HEIGHT = Meters.of(0.0);
  public static final Distance TOLERANCE = Centimeters.of(5);

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.5);
  public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(10.0);
  public static final LinearAcceleration SLOW_ACCELERATION = MetersPerSecondPerSecond.of(5.0);

  @Logged(name = "Elevator Hardware Loggables")
  private final ElevatorIO hardware;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  @Logged(name = "Filtered Parent Motor Stator Current (Amps)")
  private double filteredCurrent;

  private boolean isZeroed = false;

  public Elevator(ElevatorIO hardware) {
    this.hardware = hardware;
  }

  @Override
  public void periodic() {
    hardware.periodic();

    filteredCurrent = currentFilter.calculate(hardware.getParentStatorCurrentAmps());
  }

  /**
   * @return true if elevator has been zeroed
   */
  @Logged(name = "Is Zeroed (bool)")
  public boolean isZeroed() {
    return isZeroed;
  }

  /**
   * @return true if elevator is within a certain tolerance of the setpoint pose
   */
  @Logged(name = "Near Setpoint Pose (bool)")
  public boolean isNearSetpointPose() {
    return MathUtil.isNear(
        hardware.getSetpointPoseMeters(), hardware.getParentPoseMeters(), TOLERANCE.in(Meters));
  }

  // TODO
  @Logged(name = "Carriage (Pose3d)")
  public Pose3d getCarriagePose3d() {
    return Pose3d.kZero;
  }

  // TODO
  @Logged(name = "First Stage (Pose3d)")
  public Pose3d getFirstStagePose3d() {
    return Pose3d.kZero;
  }

  /**
   * Run elevator into hardstop to determine home (aka, zero)
   *
   * @return {@link Command}
   */
  public Command runCurrentHoming() {
    return this.run(() -> hardware.setVolts(-2.5))
        .until(() -> Math.abs(filteredCurrent) > 50.0)
        .finallyDo(
            (interupted) -> {
              if (!interupted) {
                hardware.resetEncoders(0.0);
                isZeroed = true;
              }
            });
  }

  /**
   * Run elevator to specified extension
   *
   * @param extension {@link Extension} representing desired extension
   * @return {@link Command}
   */
  public Command runExtension(Distance extension) {
    return this.run(
        () ->
            hardware.setPose(extension.in(Meters), MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
  }

  /**
   * Run elevator to specified extension
   *
   * @param extension desired extension in meters
   * @return {@link Command}
   */
  public Command runExtension(DoubleSupplier extensionMeters) {
    return this.run(
        () ->
            hardware.setPose(
                extensionMeters.getAsDouble(), MAX_ACCELERATION.in(MetersPerSecondPerSecond)));
  }

  /**
   * Run elevator to specified extension
   *
   * @param extension desired extension in meters
   * @return {@link Command}
   */
  public Command runExtension(double extensionMeters) {
    return runExtension(() -> extensionMeters);
  }

  /**
   * Run elevator to specified extension
   *
   * @param extension {@link Extension} representing desired extension
   * @return {@link Command}
   */
  public Command runSlowExtension(Distance extension) {
    return this.run(
        () ->
            hardware.setPose(extension.in(Meters), SLOW_ACCELERATION.in(MetersPerSecondPerSecond)));
  }

  /**
   * Run elevator to specified extension
   *
   * @param extension desired extension in meters
   * @return {@link Command}
   */
  public Command runSlowExtension(DoubleSupplier extension) {
    return this.run(
        () ->
            hardware.setPose(
                extension.getAsDouble(), SLOW_ACCELERATION.in(MetersPerSecondPerSecond)));
  }

  /**
   * Run elevator to specified extension
   *
   * @param extension desired extension in meters
   * @return {@link Command}
   */
  public Command runSlowExtension(double extension) {
    return runSlowExtension(() -> extension);
  }

  /**
   * Hold elevator at current extension
   *
   * @return {@link Command}
   */
  public Command holdExtension() {
    return Commands.sequence(
        this.run(
                () -> {
                  var currentPose = hardware.getParentPoseMeters();

                  hardware.setPose(currentPose, SLOW_ACCELERATION.in(MetersPerSecondPerSecond));
                })
            .until(() -> true),
        this.run(() -> {}));
  }
}
