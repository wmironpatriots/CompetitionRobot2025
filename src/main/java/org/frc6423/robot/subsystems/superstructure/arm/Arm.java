// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.lib.subsystems.RollerIO;

/** Arm Subsystem */
public class Arm extends SubsystemBase {
  @Logged(name = "Pivot Subsystem-Component")
  private final ArmPivot pivot;

  @Logged(name = "Roller Subsystem-Component")
  private final ArmRoller roller;

  @Logged(name = "Arm Setpoint State")
  private ArmState setpointState = ArmState.STOWED;

  public Arm(ArmPivotIO pivotHardware, RollerIO rollerHardware) {
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
   * Run arm to specified {@link ArmState}
   *
   * @param state desired {@link ArmState}
   * @return {@link Command}
   */
  public Command runState(ArmState state) {
    return Commands.parallel(
        this.run(() -> setpointState = state).until(() -> true),
        pivot.runAngle(state.angle.getMeasure()),
        roller.runSpeed(state.rollerSpeedRpm));
  }

  /**
   * Hold arm at current angle and roller speed
   *
   * @return {@link Command}
   */
  public Command holdState() {
    return Commands.parallel(
        this.run(() -> {}).until(() -> true), pivot.holdAngle(), roller.holdSpeed());
  }
}
