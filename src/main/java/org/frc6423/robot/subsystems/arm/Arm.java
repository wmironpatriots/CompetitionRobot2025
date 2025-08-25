// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.arm;

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

public class Arm extends SubsystemBase implements AutoCloseable {
  /** CONSTANTS */
  public static final double PIVOT_GEARING = 50;

  public static final double ROLLER_GEARING = 5.8677; // ! this is wrong

  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.001);
  public static final Distance LENGTH = Inches.of(7.5);

  public static final Angle MAX_ANGLE = Degrees.of(180);
  public static final Angle TOLERANCE = Degrees.of(1.5);

  @Logged(name = "Arm Hardware Loggables")
  private final ArmIO hardware;

  private final LinearFilter pivotCurrentFilter = LinearFilter.movingAverage(5);
  private final LinearFilter rollerCurrentFilter = LinearFilter.movingAverage(3);

  @Logged(name = "Filted Pivot Motor Current (Amps)")
  private double filteredPivotCurrent;

  @Logged(name = "Filted Roller Motor Current (Amps)")
  private double filteredRollerCurrent;

  @Logged(name = "Is Zeroed (bool)")
  private boolean isZeroed = false;

  private final Mechanism2d canvas =
      new Mechanism2d(LENGTH.in(Centimeters), LENGTH.in(Centimeters) * 2);
  private final MechanismRoot2d root =
      canvas.getRoot("pivot", LENGTH.in(Centimeters), LENGTH.in(Centimeters));
  private final MechanismLigament2d visualizer =
      root.append(
          new MechanismLigament2d(
              "arm", LENGTH.in(Centimeters), 0.0, 10, new Color8Bit(Color.kAliceBlue)));

  public Arm(ArmIO hardware) {
    this.hardware = hardware;

    SmartDashboard.putData("ArmVisualizer", canvas);
  }

  @Override
  public void periodic() {
    hardware.periodic();

    filteredPivotCurrent = pivotCurrentFilter.calculate(hardware.getPivotStatorCurrentAmps());
    filteredRollerCurrent = rollerCurrentFilter.calculate(hardware.getRollerStatorCurrentAmps());

    visualizer.setAngle(
        Rotation2d.fromRadians(hardware.getPivotAngleRads()).plus(Rotation2d.kCCW_90deg));
  }

  /**
   * @return true if arm is within a certain tolerance of the setpoint angle
   */
  @Logged(name = "Near Setpoint Angle (bool)")
  public boolean nearSetpointAngle() {
    return MathUtil.isNear(
        hardware.getPivotSetpointAngleRads(), hardware.getPivotAngleRads(), TOLERANCE.in(Radians));
  }

  /**
   * @return true when coral hits hardstop
   */
  @Logged(name = "Coral Vectored (bool)")
  public boolean coralVectored() {
    // Checks if roller motors are stalled
    // TODO check this value
    return Math.abs(filteredRollerCurrent) > 30.0;
  }

  /**
   * Run arm backwards until it hits hardstop to determine zero
   *
   * @return {@link Command}
   */
  public Command runCurrentHoming() {
    return this.run(
            () -> {
              hardware.runPivotVolts(-2.5);
              // TODO check this value
            })
        .until(() -> Math.abs(filteredPivotCurrent) > 50.0)
        .finallyDo(
            (interupted) -> {
              if (!interupted) {
                hardware.resetPivotEncoder(0.0);
                isZeroed = true;
              }
            });
  }

  /**
   * Run Arm to specified {@link ArmState}
   *
   * @param state {@link ArmState} representing hardware setpoints
   * @return {@link Command}
   */
  public Command runState(ArmState state) {
    return this.run(
        () -> {
          hardware.runPivotAngle(state.angle.getRadians());
          hardware.runRollerSpeed(state.rollerSpeedRpm);
        });
  }

  /**
   * Hold Arm at specified {@link ArmState}
   *
   * @param state {@link ArmState} representing hardware setpoints
   * @return {@link Command}
   */
  public Command holdState(ArmState state) {
    return Commands.sequence(
        this.run(
                () -> {
                  var currentAngle = hardware.getPivotAngleRads();
                  var currentSpeed = hardware.getRollerSpeedRpm();

                  hardware.runPivotAngle(currentAngle);
                  hardware.runRollerSpeed(currentSpeed);
                })
            .until(() -> true),
        this.run(() -> {}));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
