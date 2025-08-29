// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.frc6423.robot.subsystems.superstructure.elevator.Elevator.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Simulated {@link ElevatorIOReal} */
public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor model = DCMotor.getKrakenX60Foc(2);
  private final ElevatorSim sim =
      new ElevatorSim(
          model,
          GEAR_REDUCTION,
          LIFT_MASS.in(Kilograms),
          DRUM_RADIUS.in(Meters),
          0.0,
          MAX_EXTENSION_HEIGHT.in(Meters),
          true,
          0.0);

  private double appliedVolts;

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.1265, 0.4, 0.0);
  private final ProfiledPIDController feedback =
      new ProfiledPIDController(15.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.25, 10.0));

  public ElevatorIOSim() {
    SmartDashboard.putData(feedback);
  }

  @Override
  public void periodic() {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
  }

  @Override
  public double getParentPoseMeters() {
    return sim.getPositionMeters();
  }

  @Override
  public double getChildPoseMeters() {
    return sim.getPositionMeters();
  }

  @Override
  public double getSetpointPoseMeters() {
    return feedback.getSetpoint().position;
  }

  @Override
  public double getParentStatorCurrentAmps() {
    return sim.getCurrentDrawAmps();
  }

  @Override
  public double getChildStatorCurrentAmps() {
    return sim.getCurrentDrawAmps();
  }

  @Override
  public double getParentTempCelsius() {
    return 0.0;
  }

  @Override
  public double getChildTempCelsius() {
    return 0.0;
  }

  @Override
  public void resetEncoders(double poseMeters) {}

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPose(double poseMeters) {
    // Clamp pose in range
    poseMeters = MathUtil.clamp(poseMeters, 0.0, MAX_EXTENSION_HEIGHT.in(Meters));
    // Get current setpoint velocity
    var currentVel = feedback.getSetpoint().velocity;

    // Calculate next feedback setpoint
    var fbOut = feedback.calculate(getParentPoseMeters(), poseMeters);
    // Get next velocity
    var nextVel = feedback.getSetpoint().velocity;

    // Calculate feedforward velocity output
    var ffOut = feedforward.calculateWithVelocities(currentVel, nextVel);

    // Combine output
    setVolts(ffOut + fbOut);
  }

  @Override
  public void close() throws Exception {}
}
