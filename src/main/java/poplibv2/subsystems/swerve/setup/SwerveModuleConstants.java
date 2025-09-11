package poplibv2.subsystems.swerve.setup;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public int driveMotorCANID;
    public int rotMotorCANID;
    public int absEncCANID;
    public Rotation2d absEncOffset;

    /**
     * Sets some constants for a swerve module
     * @param driveMotorCANID the can id of the drive motor
     * @param rotMotorCANID the can id of the rotation motor
     * @param absEncCANID the can id of the absolute encoder
     * @param absEncOffset the offset to apply to the absolute encoder in degrees
     */
    public SwerveModuleConstants(int driveMotorCANID, int rotMotorCANID, int absEncCANID, double absEncOffset) {
        this.driveMotorCANID = driveMotorCANID;
        this.rotMotorCANID = rotMotorCANID;
        this.absEncCANID = absEncCANID;
        this.absEncOffset = Rotation2d.fromDegrees(absEncOffset);
    }
}
