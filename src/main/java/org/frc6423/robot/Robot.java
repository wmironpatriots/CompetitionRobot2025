// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import static edu.wpi.first.epilogue.Logged.Importance.INFO;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc6423.lib.drivers.CommandRobot;
import org.frc6423.robot.Constants.Flags;
import org.frc6423.robot.subsystems.arm.Arm;
import org.frc6423.robot.subsystems.arm.ArmIOSim;
import org.frc6423.robot.subsystems.arm.ArmState;

/**
 * Declares the structure of the robot program (subsystems, commands, triggers, etc.).
 *
 * <p>Only scheduler calls are allowed in the {@link Robot} periodic method. Very little logic
 * should be defined in it.
 */
@Logged
public class Robot extends CommandRobot {
  // * IO INIT
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // * SUBSYSTEM INIT
  @Logged(importance = INFO)
  private final Arm arm = new Arm(new ArmIOSim());

  // * ALERT INIT
  private final Alert batteryBrownout = new Alert("Battery voltage output low", AlertType.kWarning);

  private final Alert driverDisconnected =
      new Alert("Driver controller is Disconnected", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller is Disconnected", AlertType.kWarning);

  public Robot() {
    // Set looptime from its flag
    super(Flags.PERIOD.in(Seconds));

    // Prevent driverstation from clogging output
    DriverStation.silenceJoystickConnectionWarning(true);

    // Initialize Epilogue
    Epilogue.configure(
        config -> {
          // Set root data path
          config.root = "Telemetry";

          // Mirror NT logged data to log file
          DataLogManager.start();
          // Lazy log to NT
          config.backend =
              new LazyBackend(new NTEpilogueBackend(NetworkTableInstance.getDefault()));

          // Log build data to datalog
          final String meta = "/BuildData/";
          config.backend.log(meta + "RuntimeType", getRuntimeType().toString());
          config.backend.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
          config.backend.log(meta + "Version", BuildConstants.VERSION);
          config.backend.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
          config.backend.log(meta + "GitDirty", String.valueOf(BuildConstants.DIRTY));
          config.backend.log(meta + "GitSHA", BuildConstants.GIT_SHA);
          config.backend.log(meta + "GitDate", BuildConstants.GIT_DATE);
          config.backend.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);

          // crash sim on epilogue error
          if (isSimulation()) {
            config.errorHandler = ErrorHandler.crashOnError();
          }

          // Log everything
          config.minimumImportance = Logged.Importance.DEBUG;
        });
    Epilogue.bind(this);

    // Update drive dashboard periodically
    addPeriodic(this::updateDashboard, 0.02);

    configureGameBehavior();
    configureBindings();
  }

  /** Update all driver dashboard values on NetworkTables */
  private void updateDashboard() {
    // Update Visualizers
    SmartDashboard.putNumber("Battery Volts", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("CPU Temps", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RSL status", RobotController.getRSLState());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update Alerts
    batteryBrownout.set(RobotController.isBrownedOut());
    driverDisconnected.set(!driverController.isConnected());
    operatorDisconnected.set(!operatorController.isConnected());
  }

  /** Configure behavior during different match sections */
  private void configureGameBehavior() {
    arm.setDefaultCommand(arm.runState(ArmState.STOWED));
  }

  /** Configure all Driver & Operator controller bindings */
  private void configureBindings() {
    driverController.b().whileTrue(arm.runState(ArmState.AVOIDING));
    driverController.x().whileTrue(arm.runState(ArmState.INTAKING));
  }

  @Override
  protected Command getAutonCommand() {
    // TODO Replace placeholder
    return Commands.none();
  }
}
