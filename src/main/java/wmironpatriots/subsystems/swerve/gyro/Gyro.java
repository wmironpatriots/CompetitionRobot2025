package wmironpatriots.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/** Generalized Hardware methods for a gyro */
public abstract class Gyro {
    /**
     * @return {@link Rotation2d} representing the gyro heading
     */
    public abstract Rotation2d getRotation2d();

    /**
     * Reset gyro to specified heading
     * 
     * @param heading {@link Rotation2d} representing the heading to reset to
     */
    public abstract void reset(Rotation2d heading);
}
