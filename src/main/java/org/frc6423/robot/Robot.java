// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc6423.lib.driver.CommandRobot;

@Logged
public class Robot extends CommandRobot {
  public Robot() {
    super(kDefaultPeriod);

    // Prevent disconnected joysticks from clogging logs
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure Epilogue
    Epilogue.configure(
        config -> {
          // Set log data path
          config.root = "Telemetry";

          // Lazylog to NetworkTables
          config.backend =
              new LazyBackend(new NTEpilogueBackend(NetworkTableInstance.getDefault()));

          // Start Logging
          if (Robot.isReal()) {
            DataLogManager.start();
          }

          // Set error handling method
          if (Robot.isSimulation()) {
            config.errorHandler = ErrorHandler.crashOnError();
          } else {
            config.errorHandler = ErrorHandler.printErrorMessages();
          }

          // Set the lowest importance level to be logged
          config.minimumImportance = Importance.DEBUG;
        });

    // Bind Epilogue to robot periodic
    Epilogue.bind(this);

    // Log metadata
    final String metadataPath = "/BuildData/";
    var config = Epilogue.getConfig();
    config.backend.log(metadataPath + "RuntimeType", getRuntimeType().toString());
    config.backend.log(metadataPath + "ProjectName", BuildConstants.MAVEN_NAME);
    config.backend.log(metadataPath + "Version", BuildConstants.VERSION);
    config.backend.log(metadataPath + "GitDirty", BuildConstants.DIRTY);
    config.backend.log(metadataPath + "GitRev", BuildConstants.GIT_REVISION);
    config.backend.log(metadataPath + "GitSHA", BuildConstants.GIT_SHA);
    config.backend.log(metadataPath + "GitDate", BuildConstants.GIT_DATE);
    config.backend.log(metadataPath + "GitBranch", BuildConstants.GIT_BRANCH);
    config.backend.log(metadataPath + "BuildDate", BuildConstants.BUILD_DATE);
    config.backend.log(metadataPath + "BuildUnixTime", BuildConstants.BUILD_UNIX_TIME);

    configureBindings();
    configureGameBehavior();
  }

  /** Configure Driver & Operator controller bindings */
  public void configureBindings() {}

  /** Configure behavior during different oppmodes */
  public void configureGameBehavior() {}

  @Override
  protected Command getAutonCommand() {
    return Commands.none();
  }

  @Override
  public void close() {
    super.close();
  }
}
