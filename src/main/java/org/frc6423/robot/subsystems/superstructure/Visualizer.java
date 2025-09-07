// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Centimeters;
import static org.frc6423.robot.subsystems.superstructure.arm.Arm.*;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.MAX_EXTENSION_HEIGHT;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Visualizer {
  private static Visualizer instance;

  /**
   * @return {@link Visualizer} singleton
   */
  public static Visualizer getInstance() {
    if (instance == null) {
      instance = new Visualizer();
    }

    return instance;
  }

  private final Mechanism2d mech2d =
      new Mechanism2d(
          MAX_EXTENSION_HEIGHT.in(Centimeters), MAX_EXTENSION_HEIGHT.in(Centimeters) * 2 + 10);

  // Elevator Roots
  private final MechanismRoot2d baseRoot =
      mech2d.getRoot("base", MAX_EXTENSION_HEIGHT.in(Centimeters) / 2, 0.0);
  private final MechanismRoot2d stageRoot =
      mech2d.getRoot("stageRoot", (MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 1, 0.0);
  private final MechanismRoot2d carriageRoot =
      mech2d.getRoot("carriageRoot", (MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 2, 0.0);

  // Arm Ligament
  private final MechanismLigament2d arm;

  private Visualizer() {
    // Setup elevator ligaments
    baseRoot.append(
        new MechanismLigament2d("base", 81.43936976, 90.0, 10.0, new Color8Bit(Color.kRed)));
    stageRoot.append(
        new MechanismLigament2d("stage", 83.82, 90.0, 7.0, new Color8Bit(Color.kYellow)));
    var carriageLig =
        carriageRoot.append(
            new MechanismLigament2d("carriage", 14.083, 90.0, 4.5, new Color8Bit(Color.kGreen)));

    arm =
        carriageLig.append(
            new MechanismLigament2d(
                "arm", LENGTH.in(Centimeters), 0.0, 10, new Color8Bit(Color.kAliceBlue)));

    SmartDashboard.putData("GlobalVisualizer", mech2d);
  }

  public void setStageHeight(Distance height) {
    stageRoot.setPosition((MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 2, height.in(Centimeters));
  }

  public void setCarriageHeight(Distance height) {
    carriageRoot.setPosition(
        (MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 4, height.in(Centimeters));
  }

  public void setStageHeight(double heightCentimeters) {
    stageRoot.setPosition((MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 2, heightCentimeters);
  }

  public void setCarriageHeight(double heightCentimeters) {
    carriageRoot.setPosition((MAX_EXTENSION_HEIGHT.in(Centimeters) / 2) - 4, heightCentimeters);
  }

  public void setArmAngle(Rotation2d angle) {
    arm.setAngle(angle.unaryMinus().plus(Rotation2d.kCCW_90deg));
  }

  public void setArmAngle(double angleRads) {
    arm.setAngle(Rotation2d.fromRadians(angleRads).unaryMinus().plus(Rotation2d.kCCW_90deg));
  }
}
