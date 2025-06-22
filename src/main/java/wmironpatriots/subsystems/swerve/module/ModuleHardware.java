package wmironpatriots.subsystems.swerve.module;

import monologue.Annotations.Log;

/** Generalized Hardware methods for a Swerve Module */
public abstract class ModuleHardware {
    @Log public double pivotSetpointPoseRevs;
    @Log public double pivotPoseRevs;

    @Log public double driveSetpointSpeedMps;
    @Log public double driveSpeedMps;
    @Log public double driveDistanceMeters;

    @Log public double encoderPoseRevs;

    /**
     * @return pivot motor's position in Revolutions
     */
    public abstract double getPivotPoseRevs();

    /**
     * @return distance driven by drive motor in Meters
     */
    public abstract double getDriveDistanceMeters();

    /**
     * @return drive motor's speed in Meters Per Second
     */
    public abstract double getDriveSpeedMps();

    /**
     * Set pivot motor's voltage setpoint
     * 
     * @param volts desired voltage setpoint
     */
    public abstract void setPivotAppliedVolts(double volts);

    /**
     * Set drive motor's voltage setpoint
     * 
     * @param volts desired voltage setpoint
     */
    public abstract void setDriveAppliedVolts(double volts);

    /**
     * Set pivot motor's position setpoint
     * 
     * @param volts desired position setpoint in Revolutions
     */
    public abstract void setPivotSetpointPose(double poseRevs);

    /**
     * Set drive motor's speed setpoint
     * 
     * @param volts desired speed setpoint in MMeters Per Second
     */
    public abstract void setDriveSetpointSpeed(double speedMps);

    /** Stop both pivot and drive motors */
    public abstract void stop();
    
    /**
     * Toggle motor coasting
     * 
     * @param enabled should coasting be enabled?
     */
    public abstract void coastingEnabled(boolean enabled);
}
