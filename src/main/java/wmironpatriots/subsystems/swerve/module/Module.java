package wmironpatriots.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private final ModuleHardware hardware;

    public Module(ModuleHardware hardware) {
        this.hardware = hardware;
    }

    /**
     * Set angle and speed setpoints of the module
     *
     * @param setpointState {@link SwerveModuleState} representing the desired angle and speed of the
     *     module
     * @return {@link SwerveModuleState} representing the applied optimized setpoints
     */
    public SwerveModuleState setSetpoints(SwerveModuleState setpointState) {
        // Minimize the change in heading
        setpointState.optimize(getRotation2d());

        // Decrease drive speed based on distance to angle setpoint (reduces thread wear)
        setpointState.speedMetersPerSecond *= setpointState.angle.minus(getRotation2d()).getCos();

        hardware.setDriveSetpointSpeed(setpointState.speedMetersPerSecond);
        hardware.setPivotSetpointPose(setpointState.angle.getRotations());

        return setpointState;
    }

    /** Stop all module movement */
    public void stop() {
        hardware.stop();
    }

    /**
     * @return {@link Rotation2d} representing the angle of the module
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(hardware.getPivotPoseRevs());
    }

    /**
     * @return {@link SwerveModuleState} representing the measured position and speed of the module
     */
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(hardware.getDriveSpeedMps(), getRotation2d());
    }

    /**
     * @return {@link SwerveModulePosition} representing the measured field pose of the module
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(hardware.getDriveDistanceMeters(), getRotation2d());
    }
}