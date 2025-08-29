// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.frc6423.robot.subsystems.superstructure.arm.ArmPivot.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Simulated {@link ArmPivotIOReal} */
public class ArmPivotIOSim implements ArmPivotIO {
  private final DCMotor pivotModel = DCMotor.getKrakenX60Foc(1);
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          pivotModel,
          GEAR_REDUCTION,
          MOI.in(KilogramSquareMeters),
          LENGTH.in(Meters),
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          false,
          0);

  private double pivotAppliedVolts;

  private final ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(10, 0.0, 0.0, new TrapezoidProfile.Constraints(3.5, 3.5));

  public ArmPivotIOSim() {
    SmartDashboard.putData(pivotFeedback);
  }

  @Override
  public void periodic() {
    pivotSim.setInputVoltage(pivotAppliedVolts);
    pivotSim.update(0.02);
  }

  @Override
  public double getAngleRads() {
    return pivotSim.getAngleRads();
  }

  @Override
  public double getSetpointAngleRads() {
    return pivotFeedback.getSetpoint().position;
  }

  @Override
  public double getStatorCurrentAmps() {
    return pivotSim.getCurrentDrawAmps();
  }

  @Override
  public void resetEncoder(double poseRads) {}

  @Override
  public void setVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setAngle(double angleRads) {
    // Clamp value within range
    angleRads = MathUtil.clamp(angleRads, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));

    // Calculate fb output
    var fbOut = pivotFeedback.calculate(getAngleRads(), angleRads);

    setVolts(fbOut);
  }

  @Override
  public void close() throws Exception {}
}
