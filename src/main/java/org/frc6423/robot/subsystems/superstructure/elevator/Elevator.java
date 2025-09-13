// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.frc6423.lib.utilities.NtUtils;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.Robot;

/** Elevator Subsystem */
public class Elevator extends SubsystemBase implements AutoCloseable {
  /** Name of the CAN bus hardware is on */
  public static final String CANBUS = "CANchan";

  /** Parent motor CAN ID */
  public static final int PARENT_MOTOR_ID = 14;

  /** Child motor CAN ID */
  public static final int CHILD_MOTOR_ID = 15;

  /** Gear ratio of the elevator gearbox */
  public static final double GEAR_REDUCTION = 3 / 1;

  /** The radius of the sproket on the elevator's driven shaft */
  public static final Distance DRUM_RADIUS = Inches.of(1.757 / 2);

  /** The ratio of motor revs over output extension in meters */
  public static final double SENSOR_TO_MECH_RATIO = 3 / (2 * Math.PI * DRUM_RADIUS.in(Meters));

  /** Combined mass lifted by elevator gearbox */
  public static final Mass LIFT_MASS = Pounds.of(6.0); // TODO calculate actual value

  /** The highest feasible extension height */
  public static final Distance MAX_EXTENSION_HEIGHT = Inches.of(24);

  /** The max allowable height extension error */
  public static final Distance TOLERANCE = Inches.of(1.5);

  /** The velocity limit of the elevator's trapezoid profile */
  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.5);

  /** The acceleration of the elevator's trapezoid profile */
  public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(10.0);

  /** Represents a height the elevator can extend to */
  public static enum ElevatorExtension {
    /** Extension height for resting */
    STOWED(Inches.of(0.0)),
    /** Extension height for intaking coral */
    INTAKING(Inches.of(9.963)),
    /** Extension height for scoring on Level 2 */
    L2(Inches.of(4.549)),
    /** Extension height for scoring on Level 3 */
    L3(Inches.of(12.41)),
    /** Extension height for scoring on Level 4 */
    L4(Inches.of(24.0));

    Distance height;

    ElevatorExtension(Distance height) {
      this.height = height;
    }
  }

  @Logged(name = "Elevator Hardware Loggables")
  private final ElevatorIO hardware;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  @Logged(name = "Filtered Parent Motor Stator Current (Amps)")
  private double filteredCurrent;

  private boolean isZeroed = false;

  /** Setup Tunable Values */
  private static final String gainTopic = "Tunables/elevator";

  private static final BooleanEntry replaceGainsEntry =
      NtUtils.createBooleanEntry(gainTopic + "/replaceGains", false);
  private static final DoubleEntry KgEntry = NtUtils.createDoubleEntry(gainTopic + "/kG", 0.0);
  private static final DoubleEntry KsEntry = NtUtils.createDoubleEntry(gainTopic + "/kS", 0.0);
  private static final DoubleEntry KvEntry = NtUtils.createDoubleEntry(gainTopic + "/kV", 0.0);
  private static final DoubleEntry KaEntry = NtUtils.createDoubleEntry(gainTopic + "/kA", 0.0);
  private static final DoubleEntry KpEntry = NtUtils.createDoubleEntry(gainTopic + "/kP", 0.0);
  private static final DoubleEntry KdEntry = NtUtils.createDoubleEntry(gainTopic + "/kD", 0.0);
  private static final DoubleEntry MaxVelEntry =
      NtUtils.createDoubleEntry(gainTopic + "/maxVel", 0.0);
  private static final DoubleEntry MaxAccelEntry =
      NtUtils.createDoubleEntry(gainTopic + "/maxAccel", 0.0);

  /** Unpublish and close tunables if not in Tune Mode */
  static {
    if (!Flags.TUNE_MODE) {
      KgEntry.unpublish();
      KsEntry.unpublish();
      KvEntry.unpublish();
      KaEntry.unpublish();
      KpEntry.unpublish();
      KdEntry.unpublish();
      MaxVelEntry.unpublish();
      MaxAccelEntry.unpublish();
      KgEntry.close();
      KsEntry.close();
      KvEntry.close();
      KaEntry.close();
      KpEntry.close();
      KdEntry.close();
      MaxVelEntry.close();
      MaxAccelEntry.close();
    }
  }

  /**
   * @return fake {@link Elevator} subsystem
   */
  public static Elevator none() {
    return new Elevator(new ElevatorIONone());
  }

  /**
   * Factory for creating a {@link Elevator} subsystem
   *
   * @return {@link Elevator} Subsystem
   */
  public static Elevator create() {
    if (Robot.isReal()) {
      return new Elevator(new ElevatorIOReal());
    } else {
      return new Elevator(new ElevatorIOSim());
    }
  }

  private Elevator(ElevatorIO hardware) {
    this.hardware = hardware;
  }

  @Override
  public void periodic() {
    hardware.periodic();

    filteredCurrent = currentFilter.calculate(hardware.getParentStatorCurrentAmps());

    if (Flags.TUNE_MODE && replaceGainsEntry.getAsBoolean()) {
      hardware.setGains(
          KgEntry.get(),
          KsEntry.get(),
          KvEntry.get(),
          KaEntry.get(),
          KpEntry.get(),
          KdEntry.get(),
          MaxVelEntry.get(),
          MaxAccelEntry.get());
      replaceGainsEntry.set(false);
    }
  }

  /**
   * @return true if elevator has been homed
   */
  @Logged(name = "Is Zeroed (bool)")
  public boolean isZeroed() {
    return isZeroed;
  }

  /**
   * @return true if elevator is within a certain tolerance of the setpoint pose
   */
  @Logged(name = "Near Setpoint Pose (bool)")
  public boolean isNearSetpointPose() {
    return MathUtil.isNear(
        hardware.getSetpointPoseMeters(), hardware.getParentPoseMeters(), TOLERANCE.in(Meters));
  }

  /**
   * @return {@link Distance} representing stage height
   */
  public Distance getStageHeight() {
    return Meters.of(hardware.getParentPoseMeters());
  }

  /**
   * @return {@link Pose3d} representing the pose of the first stage
   */
  @Logged(name = "Stage (Pose3d)")
  public Pose3d getFirstPose3d() {
    return new Pose3d(Meters.of(0.0), Meters.of(0.0), getStageHeight(), Rotation3d.kZero);
  }

  /**
   * @return {@link Distance} representing carriage height
   */
  public Distance getCarriageHeight() {
    return getStageHeight().times(2);
  }

  /**
   * @return {@link Pose3d} representing the pose of the carriage
   */
  @Logged(name = "Carriage (Pose3d)")
  public Pose3d getCarriagePose3d() {
    return new Pose3d(Meters.of(0.0), Meters.of(0.0), getCarriageHeight(), Rotation3d.kZero);
  }

  /**
   * Run elevator into hardstop to determine home (aka, zero)
   *
   * @return {@link Command}
   */
  public Command runCurrentHoming() {
    return this.run(() -> hardware.setVolts(-2.5))
        .until(() -> Math.abs(filteredCurrent) > 50.0)
        .finallyDo(
            (interupted) -> {
              if (!interupted) {
                hardware.resetEncoders(0.0);
                isZeroed = true;
                System.out.println("Elevator Homed!");
              }
            });
  }

  /**
   * Run elevator to specified extension height
   *
   * @param extension {@link Distance} representing desired extension height
   * @return {@link Command}
   */
  public Command runExtension(Distance extension) {
    return this.run(() -> hardware.setPose(extension.in(Meters)));
  }

  /**
   * Run elevator to specified extension height
   *
   * @param extension {@link ElevatorExtension} representing desired extension height
   * @return {@link Command}
   */
  public Command runExtension(ElevatorExtension extension) {
    return runExtension(extension.height);
  }

  /**
   * Run elevator to specified extension height
   *
   * @param height desired extension height in meters
   * @return {@link Command}
   */
  public Command runExtension(DoubleSupplier extensionMeters) {
    return this.run(() -> hardware.setPose(extensionMeters.getAsDouble()));
  }

  /**
   * Run elevator to specified extension height
   *
   * @param height desired extension height in meters
   * @return {@link Command}
   */
  public Command runExtension(double extensionMeters) {
    return this.runExtension(() -> extensionMeters);
  }

  /**
   * Hold elevator at current extension height
   *
   * @return {@link Command}
   */
  public Command holdExtension() {
    return runExtension(() -> hardware.getParentPoseMeters());
  }

  @Override
  public void close() throws Exception {
    /** Close Tunable Entries */
    KgEntry.close();
    KsEntry.close();
    KvEntry.close();
    KaEntry.close();
    KpEntry.close();
    KdEntry.close();
    MaxVelEntry.close();
    MaxAccelEntry.close();

    hardware.close();
  }
}
