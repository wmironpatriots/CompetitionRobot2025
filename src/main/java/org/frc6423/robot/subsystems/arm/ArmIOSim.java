// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.arm;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.frc6423.robot.subsystems.arm.Arm.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOSim implements ArmIO {
  private final DCMotor pivotModel = DCMotor.getKrakenX60Foc(1);
  // TODO stddevs
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          pivotModel,
          PIVOT_GEARING,
          MOI.in(KilogramSquareMeters),
          LENGTH.in(Meters),
          0,
          MAX_ANGLE.in(Radians),
          false,
          0);

  private final DCMotor rollerModel = DCMotor.getKrakenX60(1);
  // TODO stddevs
  private final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(pivotModel, 0.01, ROLLER_GEARING), rollerModel);

  private double pivotAppliedVolts;
  private double rollerAppliedVolts;

  private final ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(10, 0.0, 0.0, new TrapezoidProfile.Constraints(3.5, 3.5));

  private final SimpleMotorFeedforward rollerFeedforward =
      new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public ArmIOSim() {
    SmartDashboard.putData(pivotFeedback);
  }

  @Override
  public void periodic() {
    pivotSim.setInputVoltage(pivotAppliedVolts);
    rollerSim.setInputVoltage(rollerAppliedVolts);

    pivotSim.update(0.02);
    rollerSim.update(0.02);
  }

  @Override
  public double getPivotAngleRads() {
    return pivotSim.getAngleRads();
  }

  @Override
  public double getPivotSetpointAngleRads() {
    return pivotFeedback.getSetpoint().position;
  }

  @Override
  public double getPivotStatorCurrentAmps() {
    return pivotSim.getCurrentDrawAmps();
  }

  @Override
  public double getRollerSpeedRpm() {
    return rollerSim.getAngularVelocityRPM();
  }

  @Override
  public double getRollerStatorCurrentAmps() {
    return rollerSim.getCurrentDrawAmps();
  }

  @Override
  public void resetPivotEncoder(double poseRads) {}

  @Override
  public void runPivotVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void runPivotAngle(double angleRads) {
    // Clamp value within range
    angleRads = MathUtil.clamp(angleRads, 0.0, MAX_ANGLE.in(Radians));

    // Calculate fb output
    var fbOut = pivotFeedback.calculate(getPivotAngleRads(), angleRads);

    runPivotVolts(fbOut);
  }

  @Override
  public void runRollerVolts(double volts) {
    rollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void runRollerSpeed(double speedRpm) {
    var ffOut = rollerFeedforward.calculate(speedRpm);
    runRollerVolts(ffOut);
  }

  @Override
  public void close() throws Exception {}
}
