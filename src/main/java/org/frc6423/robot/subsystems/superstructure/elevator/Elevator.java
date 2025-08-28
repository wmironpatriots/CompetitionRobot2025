// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/** Elevator Subsystem */
public class Elevator extends SubsystemBase {
  // * CONSTANTS
  public static final double GEAR_REDUCTION = 3 / 1;

  /** The radius of the sproket on the elevator's driven shaft */
  public static final Distance DRUM_RADIUS = Inches.of(1.757 / 2);

  public static final double SENSOR_TO_MECH_RATIO = 3 / (2 * Math.PI * DRUM_RADIUS.in(Meters));
  public static final Mass LIFT_MASS = Pounds.of(6.0); // TODO calculate actual value

  public static final Distance MAX_EXTENSION_HEIGHT = Inches.of(24);
  public static final Distance TOLERANCE = Inches.of(1.5);

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.5);
  public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(10.0);

  @Logged(name = "Elevator Hardware Loggables")
  private final ElevatorIO hardware;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  @Logged(name = "Filtered Parent Motor Stator Current (Amps)")
  private double filteredCurrent;

  private boolean isZeroed = false;

  // Visualizer
  private final Mechanism2d mech2d =
      new Mechanism2d(
          MAX_EXTENSION_HEIGHT.in(Centimeters), MAX_EXTENSION_HEIGHT.in(Centimeters) * 2 + 10);
  private final MechanismRoot2d baseRoot =
      mech2d.getRoot("base", MAX_EXTENSION_HEIGHT.in(Centimeters) / 2, 0.0);
  private final MechanismRoot2d stageRoot =
      mech2d.getRoot("stageRoot", (MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 1, 0.0);
  private final MechanismRoot2d carriageRoot =
      mech2d.getRoot("carriageRoot", (MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 2, 0.0);

  public Elevator(ElevatorIO hardware) {
    this.hardware = hardware;

    baseRoot.append(
        new MechanismLigament2d("base", 81.43936976, 90.0, 10.0, new Color8Bit(Color.kRed)));
    stageRoot.append(
        new MechanismLigament2d("stage", 83.82, 90.0, 7.0, new Color8Bit(Color.kYellow)));
    carriageRoot.append(
        new MechanismLigament2d("carriage", 17.78, 90.0, 4.5, new Color8Bit(Color.kGreen)));

    SmartDashboard.putData("Elevator Visualizer", mech2d);
  }

  @Override
  public void periodic() {
    hardware.periodic();

    filteredCurrent = currentFilter.calculate(hardware.getParentStatorCurrentAmps());

    stageRoot.setPosition(
        (MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 2, getStageHeight().in(Centimeters));
    carriageRoot.setPosition(
        (MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 4, getCarriageHeight().in(Centimeters));
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

  /**
   * @return {@link Distance} representing carriage height
   */
  public Distance getCarriageHeight() {
    return Meters.of(hardware.getParentPoseMeters()).times(2);
  }

  /**
   * @return {@link Pose3d} representing the pose of the carriage
   */
  @Logged(name = "Carriage (Pose3d)")
  public Pose3d getCarriagePose3d() {
    return new Pose3d(Meters.of(0.0), Meters.of(0.0), getStageHeight().times(2), Rotation3d.kZero);
  }

  /**
   * @return {@link Distance} representing stage height
   */
  public Distance getStageHeight() {
    return Meters.of(hardware.getParentPoseMeters());
  }

  /**
   * @return {@link Pose3d} representing the pose of the first stage
   */
  @Logged(name = "Stage (Pose3d)")
  public Pose3d getFirstPose3d() {
    return new Pose3d(Meters.of(0.0), Meters.of(0.0), getStageHeight(), Rotation3d.kZero);
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
   * Run elevator to specified extension height
   *
   * @param extension {@link ElevatorExtension} representing desired extension height
   * @return {@link Command}
   */
  public Command runExtension(ElevatorExtension extension) {
    return runExtension(extension.height);
  }

  /**
   * Run elevator to specified extension height
   *
   * @param extension {@link Distance} representing desired extension height
   * @return
   */
  public Command runExtension(Distance extension) {
    return this.run(() -> hardware.setPose(extension.in(Meters)));
  }

  /**
   * Run elevator to specified extension height
   *
   * @param height desired extension height in meters
   * @return {@link Command}
   */
  public Command runExtension(DoubleSupplier extensionMeters) {
    return this.run(() -> hardware.setPose(extensionMeters.getAsDouble()));
  }

  /**
   * Run elevator to specified extension height
   *
   * @param height desired extension height in meters
   * @return {@link Command}
   */
  public Command runExtension(double extensionMeters) {
    return runExtension(() -> extensionMeters);
  }
}
