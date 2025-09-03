package poplibv2.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Defines an structure for Gyroscope classes to follow.
 */
public abstract class Gyro {
    /**
     * This method should return the normalized yaw angle of the robot (meaning that it should fit the angle from 0 to 360). 
     * The angle should be pre-inverted if nessesary.
     * @return the Angle in Degrees
     */
    public abstract Angle getNormalizedAngle();

    /**
     * This method should return the normalized yaw angle of the robot (meaning that it should fit the angle from 0 to 360). 
     * The angle should be pre-inverted if nessesary.
     * @return the Angle in Degrees as a Rotation2d object
     */
    public abstract Rotation2d getNormalizedRotation2dAngle();

    /**
     * This method will zero out the yaw, roll, and pitch of the gyro.
     */
    public abstract void zeroGyro();

    /**
     * Allows the user to set the gyro yaw to a new angle
     * @param newAngle the new angle of the gyro in degrees as a Rotation2D object.
     */
    public abstract void setAngle(Rotation2d newAngle);

    /**
     * @return the reported roll of the gyro
     */
    public abstract Angle getRoll();

    /**
     * @return the reported pitch of the gyro
     */
    public abstract Angle getPitch();
    
    /**
     * @return the reported yaw of the gyro
     */
    public abstract Angle getYaw();

    /**
     * Gets the angluar velocity around the z-axis (the yaw)
     * @return The AngularVelocity
     */
    public abstract AngularVelocity getAngularVelo();

    /**
     * Get an angle that already has the latency of the CAN Bus taken into account for. 
     * The angle should be inverted (if nessesary) and normalized (fit from 0 to 360).
     * @return The Angle in Degrees
     */
    public abstract Angle getLatencyCompensatedAngle();
}
