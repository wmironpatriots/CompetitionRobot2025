// TODO PUT COPYRIGHT YEAR!!! Copyright (c) <YEAR> FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * @see https://github.com/wpilibsuite/allwpilib/pull/5939
 */
public abstract class CommandRobot extends TimedRobot {
  private final Timer gcTimer = new Timer();
  protected final CommandScheduler scheduler = CommandScheduler.getInstance();

  private Command autonCommand;

  public CommandRobot() {
    this(kDefaultPeriod);
  }

  /**
   * @param period looptime in seconds (default is 0.02s)
   */
  public CommandRobot(double period) {
    super(period);

    gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    // Run and profile Command Scheduler
    Tracer.traceFunc("CommandScheduler", scheduler::run);

    // RIO memory optimization
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
  public void teleopExit() {}

  @Override
  public void testInit() {
    scheduler.cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /**
   * @return {@link Command} to schedule on autonomous init
   */
  protected abstract Command getAutonCommand();
}
