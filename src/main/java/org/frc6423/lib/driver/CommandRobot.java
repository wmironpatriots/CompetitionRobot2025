// Copyright (c) !!YEAR!! FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.driver;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc6423.lib.util.Tracer;

/**
 * {@link CommandRobot} is an extension of {@link TimedRobot}
 *
 * <p>The {@link CommandRobot} class is intendended to subclassed by a user creating a Robot Program
 *
 * <p>The {@link CommandScheduler} is automatically called and traced by {@link Tracer} every period
 *
 * <p>Abstract method getAutonomousCommand() is automatically scheduled on autonomous initialization
 *
 * <p>Garbage collector automatically called every 5 seconds
 *
 * @see https://github.com/wpilibsuite/allwpilib/pull/5939
 * @see {@link TimedRobot}
 */
public abstract class CommandRobot extends TimedRobot {
  private final Timer gcTimer = new Timer();
  protected final CommandScheduler scheduler = CommandScheduler.getInstance();

  private Command autonCommand;

  /** Create new {@link CommandRobot} with a loop period of 0.02 seconds */
  public CommandRobot() {
    this(kDefaultPeriod);
  }

  /**
   * Create new {@link CommandRobot} with a specified loop period
   *
   * @param period in seconds
   */
  public CommandRobot(double period) {
    super(period);

    // Start garbage collection timer
    gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    // Run and trace Command Scheduler
    Tracer.traceFunc("CommandScheduler", scheduler::run);

    // Run garbage collector every 5 seconds
    if (gcTimer.hasElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void disabledInit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void autonomousInit() {
    autonCommand = getAutonCommand();

    if (autonCommand != null) {
      autonCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void teleopInit() {
    if (autonCommand != null) {
      autonCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    scheduler.cancelAll();
    System.gc();
  }

  /**
   * @return {@link Command} to schedule on autonomous initialization
   */
  protected abstract Command getAutonCommand();
}
