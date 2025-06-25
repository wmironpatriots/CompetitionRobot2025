// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots 
// https://github.com/FIRSTTeam6423 
//
// Open Source Software; you can modify and/or share it under the terms of 
// MIT license file in the root directory of this project 
 
package wmironpatriots.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.utils.ModuleConfig;

/** Sim version of {@link ModuleHardwareReal} */
public class ModuleHardwareSim extends ModuleHardware {
  private final DCMotor pivotModel = DCMotor.getKrakenX60Foc(1);
  private final DCMotor driveModel = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim pivotSim, driveSim;
  private double pivotAppliedVolts, driveAppliedVolts;

  private PIDController pivotFeedback = new PIDController(100.0, 0.0, 0.0);
  private PIDController driveFeedback = new PIDController(3.2, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.233, 2.02, 0.05);

  public ModuleHardwareSim(ModuleConfig moduleConfig) {
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotModel, 0.004, 150 / 7), pivotModel, 0.0, 0.0);

    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveModel, 0.025, 6.12), driveModel, 0.0, 0.0);

    pivotFeedback.enableContinuousInput(0, 0.5);
  }

  @Override
  public double getPivotPoseRevs() {
    pivotPoseRevs = pivotSim.getAngularPositionRotations();
    return pivotPoseRevs;
  }

  @Override
  public double getDriveDistanceMeters() {
    driveDistanceMeters =
        driveSim.getAngularPositionRad() * ModuleHardwareReal.WHEEL_RADIUS.in(Meters);
    return driveDistanceMeters;
  }

  @Override
  public double getDriveSpeedMps() {
    driveSpeedMps =
        driveSim.getAngularVelocityRadPerSec() * ModuleHardwareReal.WHEEL_RADIUS.in(Meters);
    return driveSpeedMps;
  }

  /**
   * Simulate friction voltage
   *
   * @param motorVoltage input voltage
   * @param frictionVoltage amount of friction
   * @return modified value
   */
  private double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }
    return motorVoltage;
  }

  @Override
  public void setPivotAppliedVolts(double volts) {
    pivotAppliedVolts = addFriction(MathUtil.clamp(volts, -12.0, 12.0), 0.25);
    pivotSim.setInputVoltage(pivotAppliedVolts);
    pivotSim.update(0.02);
  }

  @Override
  public void setDriveAppliedVolts(double volts) {
    driveAppliedVolts = addFriction(MathUtil.clamp(volts, -12.0, 12.0), 0.25);
    driveSim.setInputVoltage(driveAppliedVolts);
    driveSim.update(0.02);
  }

  @Override
  public void setPivotSetpointPose(double poseRevs) {
    setPivotAppliedVolts(pivotFeedback.calculate(pivotSim.getAngularPositionRotations(), poseRevs));
  }

  @Override
  public void setDriveSetpointSpeed(double speedMps) {
    setDriveAppliedVolts(
        driveFeedback.calculate(getDriveSpeedMps(), speedMps)
            + driveFeedforward.calculate(speedMps));
  }

  @Override
  public void stop() {
    pivotSim.setInputVoltage(0.0);
    driveSim.setInputVoltage(0.0);
  }

  @Override
  public void coastingEnabled(boolean enabled) {
    // Simulated motors can't coast lol
  }
}
