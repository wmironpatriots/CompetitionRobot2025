// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.frc6423.lib.subsystems.RollerIO;
import org.frc6423.lib.subsystems.RollerIONeo;
import org.frc6423.lib.subsystems.RollerIONone;
import org.frc6423.robot.Robot;

/** Arm Subsystem */
public class Arm extends SubsystemBase {
  @Logged(name = "Pivot Subsystem-Component")
  private final ArmPivot pivot;

  @Logged(name = "Roller Subsystem-Component")
  private final ArmRoller roller;

  @Logged(name = "Arm Setpoint State")
  private ArmState setpointState = ArmState.STOWED;

  /**
   * @return fake {@link Arm} subsystem
   */
  public static Arm none() {
    return new Arm(new ArmPivotIONone(), new RollerIONone());
  }

  /**
   * Factory for creating a {@link Arm} subsystem
   *
   * @return {@link Arm} Subsystem
   */
  public static Arm create() {
    if (Robot.isReal()) {
      return new Arm(new ArmPivotIOReal(), new RollerIONeo());
    } else {
      return new Arm(new ArmPivotIOSim(), new RollerIONone());
    }
  }

  private Arm(ArmPivotIO pivotHardware, RollerIO rollerHardware) {
    this.pivot = new ArmPivot(pivotHardware);
    this.roller = new ArmRoller(rollerHardware);
  }

  @Override
  public void periodic() {}

  /**
   * @return true if arm is within a certain tolerance of the setpoint angle
   */
  public boolean isNearSetpointAngle() {
    return pivot.isNearSetpointAngle();
  }

  /**
   * @return true if arm has vectored coral piece
   */
  public boolean hasCoralVectored() {
    return roller.isStalling();
  }

  /**
   * Run arm to specified state
   *
   * @param pivotAngle {@link Angle} representing desired pivot angle
   * @param rollerSpeed {@link AngularVelocity} representing desired roller speed
   * @return {@link Command}
   */
  public Command runState(Angle pivotAngle, AngularVelocity rollerSpeed) {
    return Commands.parallel(
        this.run(() -> {}).until(() -> true),
        pivot.runAngle(pivotAngle.in(Radians)),
        roller.runSpeed(rollerSpeed.in(RPM)));
  }

  /**
   * Run arm to specified state
   *
   * @param pivotAngleRads desired pivot angle in radians
   * @param rollerSpeed desired roller speed in radians
   * @return {@link Command}
   */
  public Command runState(DoubleSupplier pivotAngleRads, DoubleSupplier rollerSpeedRpm) {
    return Commands.parallel(
        this.run(() -> {}).until(() -> true),
        pivot.runAngle(pivotAngleRads.getAsDouble()),
        roller.runSpeed(rollerSpeedRpm.getAsDouble()));
  }

  /**
   * Run arm to specified state
   *
   * @param pivotAngleRads desired pivot angle in radians
   * @param rollerSpeed desired roller speed in radians
   * @return {@link Command}
   */
  public Command runState(double pivotAngleRads, double rollerSpeedRpm) {
    return this.runState(() -> pivotAngleRads, () -> rollerSpeedRpm);
  }

  /**
   * Run arm to specified {@link ArmState}
   *
   * @param state desired {@link ArmState}
   * @return {@link Command}
   */
  public Command runState(ArmState state) {
    return this.runState(state.angle.getRadians(), state.rollerSpeedRpm);
  }
}
