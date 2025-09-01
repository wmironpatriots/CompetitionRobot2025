// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.frc6423.lib.subsystems.Roller;
import org.frc6423.lib.subsystems.RollerIO;
import org.frc6423.lib.subsystems.RollerIONeo;
import org.frc6423.lib.subsystems.RollerIONone;
import org.frc6423.robot.Robot;

/** {@link Roller} subsystem representing the {@link Arm} subsystem's rollers */
public class ArmRoller extends Roller {
  /** CONSTANTS */
  public static final double MINIMUM_STALL_CURRENT_AMPS = 30.0;

  // TODO check
  public static final double INTAKING_SPEED = 4.0;

  /**
   * @return fake {@link Roller} subsyste
   */
  public static ArmRoller none() {
    return new ArmRoller(new RollerIONone());
  }

  /**
   * Factory for creating a {@link Arm} subsystem
   *
   * @return {@link Arm} Subsystem
   */
  public static ArmRoller create() {
    if (Robot.isReal()) {
      return new ArmRoller(new RollerIONeo());
    } else {
      return new ArmRoller(new RollerIONone());
    }
  }

  public ArmRoller(RollerIO hardware) {
    super("ArmRoller", hardware);
  }

  /**
   * @return true if roller motor is stalling
   */
  public boolean isStalling() {
    return super.isStalling(MINIMUM_STALL_CURRENT_AMPS);
  }

  /**
   * Run roller at specified volts
   *
   * @param volts desired volts
   * @return {@link Command}
   */
  public Command runVolts(DoubleSupplier volts) {
    return super.runVolts(volts);
  }

  /**
   * Run roller at specified volts
   *
   * @param volts desired volts
   * @return {@link Command}
   */
  public Command runVolts(double volts) {
    return super.runVolts(volts);
  }

  /**
   * Run roller at specified speed
   *
   * @param speedRpm desired speed in revs per minute
   * @return {@link Command}
   */
  public Command runSpeed(DoubleSupplier speedRpm) {
    return super.runSpeed(speedRpm);
  }

  /**
   * Run roller at specified speed
   *
   * @param speedRpm desired speed in revs per minute
   * @return {@link Command}
   */
  public Command runSpeed(double speedRpm) {
    return super.runSpeed(speedRpm);
  }

  /**
   * Hold roller at current speed
   *
   * @return {@link Command}
   */
  public Command holdSpeed() {
    return super.holdSpeed();
  }
}
