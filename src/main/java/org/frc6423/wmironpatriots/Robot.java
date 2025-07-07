// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.wmironpatriots;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc6423.lib.utils.Tracer;
import org.frc6423.lib.wpilibExt.CommandRobot;
import org.frc6423.monologue.Logged;
import org.frc6423.monologue.Monologue;
import org.frc6423.monologue.Monologue.MonologueConfig;
import org.frc6423.wmironpatriots.Constants.FLAGS;
import org.frc6423.wmironpatriots.subsystems.swerve.Swerve;

public class Robot extends CommandRobot implements Logged {
  // HARDWARE
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final Swerve swerve = Swerve.create();

  // ALERTS
  private final Alert browningOut;

  public Robot() {
    super(FLAGS.LOOPTIME.in(Seconds));

    // Shut up driverstation
    DriverStation.silenceJoystickConnectionWarning(true);

    // ! KEEP THIS LINE
    // Signal Logger will auto enable on FMS which causes massive delay
    SignalLogger.enableAutoLogging(false);

    // Init Monologue
    Monologue.setupMonologue(
        this,
        "/Robot",
        new MonologueConfig()
            .withAllowNonFinalLoggedFields(true)
            .withDatalogPrefix("Telemetry")
            .withLazyLogging(true)
            .withOptimizeBandwidth(true) // ! .withOptimizeBandwidth(DriverStation::isFMSAttached)
            .withThrowOnWarning(false));

    // Log build data to datalog
    final String meta = "/BuildData/";
    Monologue.log(meta + "RuntimeType", getRuntimeType().toString());
    Monologue.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
    Monologue.log(meta + "Version", BuildConstants.VERSION);
    Monologue.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
    Monologue.log(meta + "GitDirty", String.valueOf(BuildConstants.DIRTY));
    Monologue.log(meta + "GitSHA", BuildConstants.GIT_SHA);
    Monologue.log(meta + "GitDate", BuildConstants.GIT_DATE);
    Monologue.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);

    // Update Monologue periodically
    addPeriodic(() -> Tracer.traceFunc("Monologue", Monologue::updateAll), 0.02);

    // Update Smart Dashboard visualizers periodically
    addPeriodic(
        () -> {
          SmartDashboard.putNumber("Battery Volts", RobotController.getBatteryVoltage());
          SmartDashboard.putNumber("CPU Temps", RobotController.getCPUTemp());
          SmartDashboard.putBoolean("RSL status", RobotController.getRSLState());
          SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        },
        0.2);

    // Set up alert triggers
    browningOut = new Alert("Browning Out!", AlertType.kWarning);
    new Trigger(() -> RobotController.isBrownedOut())
        .onTrue(Commands.run(() -> browningOut.set(true)));

    configureBindings();
    configureGameBehavior();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFromMagnitudes(driver::getLeftY, driver::getLeftX, driver::getRightX));
  }

  private void configureGameBehavior() {}

  @Override
  protected Command getAutonCommand() {
    // TODO replace placeholder
    return Commands.none();
  }
}
