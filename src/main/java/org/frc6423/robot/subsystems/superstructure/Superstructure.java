// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.robot.subsystems.superstructure.arm.Arm;
import org.frc6423.robot.subsystems.superstructure.elevator.Elevator;

public class Superstructure extends SubsystemBase {
  @Logged private final Elevator elevator = Elevator.create();
  @Logged private final Arm arm = Arm.create();

  private final Visualizer visualizer = Visualizer.getInstance();

  public Superstructure() {}

  public void periodic() {
    /** Set visualizer poses */
    visualizer.setCarriageHeight(elevator.getCarriageHeight().in(Centimeters));
    visualizer.setStageHeight(elevator.getStageHeight().in(Centimeters));
    visualizer.setArmAngle(arm.getRotation2d());
  }
}
