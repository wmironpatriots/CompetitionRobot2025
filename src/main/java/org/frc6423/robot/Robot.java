// TODO PUT COPYRIGHT YEAR!!! Copyright (c) <YEAR> FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc6423.lib.CommandRobot;
import org.frc6423.lib.Tracer;
import org.frc6423.monologue.Logged;
import org.frc6423.monologue.Monologue;
import org.frc6423.monologue.Monologue.MonologueConfig;
import org.frc6423.robot.Constants.Flags;

/**
 * Declares the structure of the robot program (subsystems, commands, triggers, etc.).
 *
 * <p>Only scheduler calls are allowed in the {@link Robot} periodic method. Very little logic
 * should be defined in it.
 */
public class Robot extends CommandRobot implements Logged {
  // * IO INIT
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  // * SUBSYSTEM INIT
  /** Replace this line with subsystem declerations */

  // * TRIGGER INIT
  /** Replace this line with trigger declerations */

  // * ALERT INIT
  private final Alert batteryBrownout = new Alert("Battery voltage output low", AlertType.kWarning);

  private final Alert driverDisconnected =
      new Alert("Driver controller is Disconnected", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller is Disconnected", AlertType.kWarning);

  public Robot() {
    // Set looptime from its flag
    super(Flags.LOOPTIME.in(Seconds));

    // Prevent driverstation from clogging output
    DriverStation.silenceJoystickConnectionWarning(true);

    // Initialize Monologue
    Monologue.setupMonologue(
        this,
        "/Robot",
        new MonologueConfig()
            .withAllowNonFinalLoggedFields(true)
            .withDatalogPrefix("Telemetry")
            .withLazyLogging(true)
            .withOptimizeBandwidth(DriverStation::isFMSAttached)
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

    // Update and trace monologue periodically
    addPeriodic(
        () -> Tracer.traceFunc("Monologue", Monologue::updateAll), Flags.LOOPTIME.in(Seconds));

    // Update drive dashboard periodically
    addPeriodic(this::updateDashboard, 0.02);

    configureBindings();
    configureGameBehavior();
  }

  /** Update all driver dashboard values on NetworkTables */
  private void updateDashboard() {
    batteryBrownout.set(RobotController.isBrownedOut());
    driverDisconnected.set(!driverController.isConnected());
    operatorDisconnected.set(!operatorController.isConnected());
  }

  /** Configure all Driver & Operator controller bindings */
  private void configureBindings() {}

  /** Configure behavior during different match sections */
  private void configureGameBehavior() {}

  @Override
  protected Command getAutonCommand() {
    // TODO Replace placeholder
    return Commands.none();
  }
}
