// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots 
// https://github.com/wmironpatriots 
//
// Open Source Software; you can modify and/or share it under the terms of 
// MIT license file in the root directory of this project 
 
package wmironpatriots.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Distance;
import lib.utils.ModuleConfig;

/**
 * MK4i /w l3 ratio
 *
 * <p>Kraken x60 for pivot
 *
 * <p>Kraken x44 for drive
 *
 * <p>CANcoder absolute encoder
 */
public class ModuleHardwareReal extends ModuleHardware {
  public static final double PIVOT_REDUCTION = 150 / 7;
  public static final double DRIVE_REDUCTION = 6.12;
  public static final Distance WHEEL_RADIUS = Inches.of(3);

  private final TalonFX pivot, drive;
  private final CANcoder cancoder;

  private final TalonFXConfiguration pivotCfg, driveCfg;
  private final CANcoderConfiguration cancoderCfg;

  private final VoltageOut voltOutReq = new VoltageOut(0.0);
  private final PositionTorqueCurrentFOC poseOutReq = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velOutReq = new VelocityTorqueCurrentFOC(0.0);

  private final BaseStatusSignal pivotPose;
  private final BaseStatusSignal drivePose, driveSpeed;
  private final BaseStatusSignal cancoderPose;

  public ModuleHardwareReal(ModuleConfig moduleConfig) {
    pivot = new TalonFX(moduleConfig.pivotId().getCanId(), moduleConfig.pivotId().getBusName());
    drive = new TalonFX(moduleConfig.driveId().getCanId(), moduleConfig.driveId().getBusName());
    cancoder =
        new CANcoder(moduleConfig.encoderId().getCanId(), moduleConfig.encoderId().getBusName());

    // Pivot Configs
    pivotCfg = new TalonFXConfiguration();

    pivotCfg.MotorOutput.Inverted =
        moduleConfig.pivotInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    pivotCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotCfg.CurrentLimits.StatorCurrentLimit = 40.0;
    pivotCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotCfg.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    pivotCfg.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    pivotCfg.TorqueCurrent.TorqueNeutralDeadband = 0.0;
    pivotCfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    pivotCfg.ClosedLoopGeneral.ContinuousWrap = true;
    pivotCfg.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderId().getCanId();
    pivotCfg.Feedback.FeedbackRotorOffset = moduleConfig.encoderOffsetRevs();
    pivotCfg.Feedback.RotorToSensorRatio = PIVOT_REDUCTION;
    pivotCfg.Feedback.SensorToMechanismRatio = 0.0;
    pivotCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    pivotCfg.Slot0.kP = 600.0;
    pivotCfg.Slot0.kD = 50.0;
    pivotCfg.Slot0.kA = 0.0;
    pivotCfg.Slot0.kV = 10;
    pivotCfg.Slot0.kS = 0.014;

    pivot.getConfigurator().apply(pivotCfg);

    // Drive Configs
    driveCfg = new TalonFXConfiguration();

    driveCfg.MotorOutput.Inverted =
        moduleConfig.driveInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveCfg.CurrentLimits.StatorCurrentLimit = 120.0;
    driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    driveCfg.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    driveCfg.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    driveCfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    driveCfg.ClosedLoopGeneral.ContinuousWrap = true;
    driveCfg.Feedback.SensorToMechanismRatio =
        DRIVE_REDUCTION / (WHEEL_RADIUS.in(Meters) * 2 * Math.PI);
    driveCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    driveCfg.Slot0.kP = 35.0;
    driveCfg.Slot0.kD = 0.0;
    driveCfg.Slot0.kA = 0.0;
    driveCfg.Slot0.kV = 0.0;
    driveCfg.Slot0.kS = 5.0;

    drive.getConfigurator().apply(driveCfg);

    // CANcoder configs
    cancoderCfg = new CANcoderConfiguration();

    cancoderCfg.MagnetSensor.MagnetOffset = moduleConfig.encoderOffsetRevs();
    cancoderCfg.MagnetSensor.SensorDirection =
        moduleConfig.pivotInverted()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    cancoder.getConfigurator().apply(cancoderCfg);

    // Status Signals
    pivotPose = pivot.getPosition();

    drivePose = drive.getPosition();
    driveSpeed = drive.getVelocity();

    cancoderPose = cancoder.getAbsolutePosition();
  }

  @Override
  public double getPivotPoseRevs() {
    encoderPoseRevs = cancoderPose.getValueAsDouble();
    pivotPoseRevs = pivotPose.getValueAsDouble();
    return pivotPoseRevs;
  }

  @Override
  public double getDriveSpeedMps() {
    driveSpeedMps = driveSpeed.getValueAsDouble();
    return driveSpeedMps;
  }

  @Override
  public double getDriveDistanceMeters() {
    driveDistanceMeters = drivePose.getValueAsDouble();
    return driveDistanceMeters;
  }

  @Override
  public void setPivotAppliedVolts(double volts) {
    pivot.setControl(voltOutReq.withOutput(volts));
  }

  @Override
  public void setDriveAppliedVolts(double volts) {
    drive.setControl(voltOutReq.withOutput(volts));
  }

  @Override
  public void setPivotSetpointPose(double poseRevs) {
    pivot.setControl(poseOutReq.withPosition(poseRevs));
  }

  @Override
  public void setDriveSetpointSpeed(double speedMps) {
    drive.setControl(velOutReq.withVelocity(speedMps));
  }

  @Override
  public void stop() {
    pivot.stopMotor();
    drive.stopMotor();
  }

  @Override
  public void coastingEnabled(boolean enabled) {
    var neutralMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    pivotCfg.MotorOutput.NeutralMode = neutralMode;
    driveCfg.MotorOutput.NeutralMode = neutralMode;

    pivot.getConfigurator().apply(pivotCfg);
    drive.getConfigurator().apply(driveCfg);
  }
}
