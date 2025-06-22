package lib.utils;

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