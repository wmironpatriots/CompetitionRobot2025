// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/** Generic Roller Subsystem */
public class Roller extends SubsystemBase {
  @Logged(name = "Roller Hardware Loggables")
  private final RollerIO hardware;

  // TODO check value
  private LinearFilter currentFilter = LinearFilter.movingAverage(3);

  @Logged(name = "Filtered Roller Motor Current (Amps)")
  private double filteredCurrent = 0.0;

  public Roller(String name, RollerIO hardware) {
    this.setName(name);
    this.hardware = hardware;
  }

  @Override
  public void periodic() {
    hardware.periodic();

    filteredCurrent = currentFilter.calculate(hardware.getStatorCurrentAmps());
  }

  /**
   * @param minStallAmps minimum roller motor stator current considered stalling in amps
   * @return true if roller motor is stalling
   */
  protected boolean isStalling(double minStallAmps) {
    return Math.abs(filteredCurrent) > 30.0;
  }

  protected void setCurrentFilterTaps(int taps) {
    currentFilter = LinearFilter.movingAverage(taps);
  }

  /**
   * Run roller at specified volts
   *
   * @param volts desired volts
   * @return {@link Command}
   */
  protected Command runVolts(DoubleSupplier volts) {
    return this.run(() -> hardware.setVolts(volts.getAsDouble()));
  }

  /**
   * Run roller at specified volts
   *
   * @param volts desired volts
   * @return {@link Command}
   */
  protected Command runVolts(double volts) {
    return runVolts(() -> volts);
  }

  /**
   * Run roller at specified speed
   *
   * @param speedRpm desired speed in revs per minute
   * @return {@link Command}
   */
  protected Command runSpeed(DoubleSupplier speedRpm) {
    return this.run(() -> hardware.setSpeed(speedRpm.getAsDouble()));
  }

  /**
   * Run roller at specified speed
   *
   * @param speedRpm desired speed in revs per minute
   * @return {@link Command}
   */
  protected Command runSpeed(double speedRpm) {
    return runSpeed(() -> speedRpm);
  }
}
