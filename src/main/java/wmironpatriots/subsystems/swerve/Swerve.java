// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots 
// https://github.com/FIRSTTeam6423 
//
// Open Source Software; you can modify and/or share it under the terms of 
// MIT license file in the root directory of this project 
 
package wmironpatriots.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;
import wmironpatriots.Constants.FLAGS;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.swerve.gyro.Gyro;
import wmironpatriots.subsystems.swerve.gyro.GyroPigeon;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.ModuleHardwareReal;
import wmironpatriots.subsystems.swerve.module.ModuleHardwareSim;

/** Swerve Subsystem Class */
public class Swerve implements Subsystem {
  public static Swerve create() {
    var moduleConfigs = SwerveConstants.MODULE_CONFIGS;
    var modules = new Module[moduleConfigs.length];

    if (Robot.isReal()) {
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new Module(new ModuleHardwareReal(moduleConfigs[i]));
      }

      return new Swerve(new GyroPigeon(MATRIXID.PIGEON), modules);
    } else {
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new Module(new ModuleHardwareSim(moduleConfigs[i]));
      }

      return new Swerve(new GyroPigeon(MATRIXID.PIGEON), modules);
    }
  }

  private final Module[] modules;
  private final Gyro gyro;

  private Rotation2d simHeading = Rotation2d.kZero;

  private final SwerveDriveKinematics kinematics = SwerveConstants.KINEMATICS;
  private final SwerveDrivePoseEstimator odometry;

  private final Field2d f2d = new Field2d();
  private final StructArrayPublisher<SwerveModuleState> swervePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
          .publish();
  private final StructArrayPublisher<SwerveModuleState> setpoint =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("SwerveStateSetpoints", SwerveModuleState.struct)
          .publish();

  private Swerve(Gyro gyro, Module... modules) {
    this.modules = modules;
    this.gyro = gyro;

    SmartDashboard.putData("SwerveField", f2d);

    odometry =
        new SwerveDrivePoseEstimator(
            kinematics, gyro.getRotation2d(), getSwerveModulePositions(), new Pose2d());
  }

  @Override
  public void periodic() {
    odometry.update(getGyroHeading(), getSwerveModulePositions());
    f2d.setRobotPose(getPose2d());
    swervePublisher.set(getSwerveModuleStates());

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  @Override
  public void simulationPeriodic() {
    var angularRate = getChassisSpeeds().omegaRadiansPerSecond;
    simHeading =
        simHeading.rotateBy(
            Rotation2d.fromRadians(
                !Double.isNaN(angularRate) ? angularRate * FLAGS.LOOPTIME.in(Seconds) : 0));
  }

  /**
   * Sets {@link SwerveModuleState} setpoints for all modules
   *
   * @param states {@link SwerveModuleState} setpoints array
   */
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    setpoint.set(states);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setSetpoints(states[i]);
    }
  }

  /**
   * Set {@link ChassisSpeeds} setpoint
   *
   * @param speeds {@link ChassisSpeeds} setpoint
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    // https://github.com/wpilibsuite/allwpilib/issues/7332
    speeds = ChassisSpeeds.discretize(speeds, FLAGS.LOOPTIME.in(Seconds));

    System.out.println(speeds.vxMetersPerSecond);

    var states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond));

    setSwerveModuleStates(states);
  }

  /** Stops all modules */
  public void stop() {
    for (Module module : modules) {
      module.stop();
    }
  }

  /**
   * Drives robot from magnitudes of max speed
   *
   * @param xSpeedMagnitude Input stream representing X speed magnitude
   * @param ySpeedMagnitude Input stream representing X speed magnitude
   * @param angularRateMagnitude Input stream representing X speed magnitude
   * @return {@link Command}
   */
  public Command driveFromMagnitudes(
      DoubleSupplier xSpeedMagnitude,
      DoubleSupplier ySpeedMagnitude,
      DoubleSupplier angularRateMagnitude) {
    return this.run(
        () ->
            setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedMagnitude.getAsDouble()
                        * SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                    ySpeedMagnitude.getAsDouble()
                        * SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                    angularRateMagnitude.getAsDouble()
                        * SwerveConstants.MAX_ANGULAR_RATE.in(RadiansPerSecond),
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? getRotation2d()
                        : getRotation2d().plus(Rotation2d.k180deg))));
  }

  /**
   * Configure wheels into an X shape and stop
   *
   * @return {@link Command}
   */
  public Command stopAndLock() {
    return this.runOnce(
        () -> {
          var states =
              new SwerveModuleState[] {
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
              };

          setSwerveModuleStates(states);
        });
  }

  /**
   * @return {@link SwerveModuleState} array of all current swerve module states
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    var moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = modules[i].getSwerveModuleState();
    }

    return moduleStates;
  }

  /**
   * @return {@link SwerveModulePosition} array of all current swerve module positions
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    var modulePoses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modulePoses.length; i++) {
      modulePoses[i] = modules[i].getSwerveModulePosition();
    }

    return modulePoses;
  }

  /**
   * @return {@link Rotation2d} representing gyro reading from last reset
   */
  public Rotation2d getGyroHeading() {
    return Robot.isReal() ? gyro.getRotation2d() : simHeading;
  }

  /**
   * @return a field relative {@link ChassisSpeeds}
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * @return {@link Pose2d} representing odometry estimated position
   */
  public Pose2d getPose2d() {
    return odometry.getEstimatedPosition();
  }

  /**
   * @return {@link Rotation2d} representing the odometry estimated heading
   */
  public Rotation2d getRotation2d() {
    return getPose2d().getRotation();
  }
}
