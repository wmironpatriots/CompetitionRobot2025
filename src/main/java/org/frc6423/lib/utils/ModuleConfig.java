// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.lib.utils;

/**
 * Represents the constants of a single module
 *
 * @param index Module identifier
 * @param pivotId Pivot motor CAN ID
 * @param driveId Drive motor CAN ID
 * @param encoderId Encoder CAN/PWM ID
 * @param encoderOffsetRevs Encoder measurement offset in Revs
 * @param pivotInverted Is pivot motor inverted?
 * @param driveInverted Is drive motor inverted?
 */
public record ModuleConfig(
    int index,
    CanDeviceId pivotId,
    CanDeviceId driveId,
    CanDeviceId encoderId,
    double encoderOffsetRevs,
    boolean pivotInverted,
    boolean driveInverted) {}
