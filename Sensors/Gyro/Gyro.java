package POPLib.Sensors.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Defines an structure for Gyroscope classes to follow.
 */
public abstract class Gyro {
    public abstract Rotation2d getAngle();

    public abstract void zeroGyro();

    public abstract void setAngle(Rotation2d newAngle);

    public abstract Rotation2d getRoll();

    public abstract Rotation2d getPitch();

    public abstract Rotation2d getYaw();

    public abstract Rotation2d getAngularVelo();

    public abstract Rotation2d getLatencyCompensatedAngle();
}
