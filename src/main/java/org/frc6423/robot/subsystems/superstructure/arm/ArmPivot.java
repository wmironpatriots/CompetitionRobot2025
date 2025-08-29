// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/** Pivot subsystem-component of the {@link Arm} Subsystem */
public class ArmPivot extends SubsystemBase implements AutoCloseable {
  /** CONSTANTS */
  public static final double PIVOT_GEARING = 50;

  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.23381);
  public static final Distance LENGTH = Inches.of(7.5);

  public static final Angle MIN_ANGLE = Degrees.of(-90);
  public static final Angle MAX_ANGLE = Degrees.of(90);
  public static final Angle TOLERANCE = Degrees.of(1.5);

  @Logged(name = "Arm Pivot Hardware Loggables")
  private final ArmPivotIO hardware;

  private final LinearFilter pivotCurrentFilter = LinearFilter.movingAverage(5);

  @Logged(name = "Filted Pivot Motor Stator Current (Amps)")
  private double filteredPivotCurrent;

  private boolean isZeroed = false;

  private final Mechanism2d canvas =
      new Mechanism2d(LENGTH.in(Centimeters), LENGTH.in(Centimeters) * 2);
  private final MechanismRoot2d root =
      canvas.getRoot("pivot", LENGTH.in(Centimeters), LENGTH.in(Centimeters));
  private final MechanismLigament2d visualizer =
      root.append(
          new MechanismLigament2d(
              "arm", LENGTH.in(Centimeters), 0.0, 10, new Color8Bit(Color.kAliceBlue)));

  public ArmPivot(ArmPivotIO hardware) {
    this.hardware = hardware;

    SmartDashboard.putData("ArmVisualizer", canvas);
  }

  @Override
  public void periodic() {
    hardware.periodic();

    filteredPivotCurrent = pivotCurrentFilter.calculate(hardware.getStatorCurrentAmps());

    visualizer.setAngle(
        Rotation2d.fromRadians(hardware.getAngleRads()).unaryMinus().rotateBy(Rotation2d.k180deg));
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
   * Run arm into hardstop to determine home (aka, zero)
   *
   * @return {@link Command}
   */
  // TODO CHECK VALUES
  public Command runCurrentHoming() {
    return this.run(() -> hardware.setVolts(-2.5))
        .until(() -> Math.abs(filteredPivotCurrent) > 50.0)
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
    return runAngle(() -> angleRads);
  }

  /**
   * Hold arm at current angle
   *
   * @return {@link Command}
   */
  public Command holdAngle() {
    return Commands.sequence(
        this.run(
                () -> {
                  var currentAngle = hardware.getAngleRads();
                  hardware.setAngle(currentAngle);
                })
            .until(() -> true),
        this.run(() -> {}));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
