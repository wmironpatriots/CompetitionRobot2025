// TODO PUT COPYRIGHT YEAR!!! Copyright (c) <YEAR> FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib;

/** Represents a boolean that will have a final value of true once toggled */
public class Latch {
  private boolean value = false;

  /**
   * @return {@link boolean} representing latch value
   */
  public boolean getValue() {
    return value;
  }

  /** Toggle latch to true */
  public void toggle() {
    value = true;
  }
}
